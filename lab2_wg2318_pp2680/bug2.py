#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi, atan, cos, sin
from sensor_msgs.msg import LaserScan
import math
import numpy as np


def scan_callback(msg):
    global object_range_ahead
    global range_ahead
    global right_ahead
    global left_ahead
    ranges = [x if not math.isnan(x) else float("inf") for x in msg.ranges]
    object_range_ahead = min(ranges)
    range_ahead = ranges[len(ranges) / 2]
    left_ahead = min(ranges[len(ranges) / 2 :])
    right_ahead = min(ranges[: len(ranges) / 2])


# Give the node a name
rospy.init_node("bugger", anonymous=False)

# Initiate global variable that indicates how far the closest object is
object_range_ahead = 1

# Initiate global variable that indicates how far the object is in front of it, right, and left
range_ahead = 1
right_ahead = 1
left_ahead = 1

# Subscriber to listen to messages from LaserScan
scan_sub = rospy.Subscriber("scan", LaserScan, scan_callback)

# Publisher to control the robot's speed
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

# How fast will we update the robot's movement?
rate = 20

# Set the equivalent ROS rate variable
r = rospy.Rate(rate)

# Set the forward linear speed to 0.15 meters per second
linear_speed = 0.5

# Set position threshold
threshold = 0.5

# Set the travel distance in meters
goal_distance = 0.5

# Set the rotation speed in radians per second
angular_speed = 1

# Set the angular tolerance in degrees converted to radians
angular_tolerance = radians(2.5)

# Set the rotation angle to Pi radians (180 degrees)
goal_angle = radians(15)

# Initialize the tf listener
tf_listener = tf.TransformListener()

# Give tf some time to fill its buffer
rospy.sleep(2)

# Set the odom frame
odom_frame = "/odom"

# Find out if the robot uses /base_link or /base_footprint
try:
    # /base_footprint
    tf_listener.waitForTransform(
        odom_frame, "/base_footprint", rospy.Time(), rospy.Duration(1.0)
    )
    base_frame = "/base_footprint"
except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    try:
        # /base_link
        tf_listener.waitForTransform(
            odom_frame, "/base_link", rospy.Time(), rospy.Duration(1.0)
        )
        base_frame = "/base_link"
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo(
            "Cannot find transform between /odom and /base_link or /base_footprint"
        )
        rospy.signal_shutdown("tf Exception")


def get_odom():
    # Get the current transform between the odom and base frames
    try:
        (trans, rot) = tf_listener.lookupTransform(
            odom_frame, base_frame, rospy.Time(0)
        )
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

    return (Point(*trans), quat_to_angle(Quaternion(*rot)))


def translate(sign):
    # Initialize the movement command
    move_cmd = Twist()

    # Set the movement command to forward motion
    move_cmd.linear.x = linear_speed * sign

    # Get the starting position values
    position = get_odom()[0]

    x_start = position.x
    y_start = position.y

    # Keep track of the distance traveled, buffer
    distance = 0

    # Enter the loop to move along a side
    while distance < goal_distance and not rospy.is_shutdown():
        # Publish the Twist message and sleep 1 cycle
        cmd_vel_pub.publish(move_cmd)

        r.sleep()
        position = get_odom()[0]

        # Compute the Euclidean distance from the start
        distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))

    position = get_odom()[0]
    position_coordinate = (position.x, position.y)
    print("Moving robot...:", position_coordinate)


def rotate(sign):
    # Initialize the movement command
    move_cmd = Twist()

    # Track the last angle measured
    last_angle = get_odom()[1]
    # Track how far we have turned
    turn_angle = 0

    # Turn right
    move_cmd.angular.z = angular_speed * sign
    while (
        abs(turn_angle + angular_tolerance) < abs(goal_angle)
        and not rospy.is_shutdown()
    ):
        cmd_vel_pub.publish(move_cmd)
        r.sleep()
        # Get the current rotation
        rotation = get_odom()[1]

        # Compute the amount of rotation since the last loop
        delta_angle = normalize_angle(rotation - last_angle)

        # Add to the running total
        turn_angle += delta_angle
        last_angle = rotation

    rotation = get_odom()[1]
    print("Rotating robot...:", rotation)


def convert_positive_radian(sign_radian):
    # convert all radians to be positive
    if sign_radian < 0:
        return 2 * pi + sign_radian
    else:
        return sign_radian


def find_radians_towards_goal(goal, position_coordinate):
    """
    Returns the sign radian direction to the goal
    """
    delta_x = goal[0] - position_coordinate[0]
    delta_y = goal[1] - position_coordinate[1]
    sign = delta_y / abs(delta_y)
    if delta_x > 0:
        return atan(delta_y / delta_x)
    elif delta_x < 0:
        return sign * (pi - atan(delta_y / delta_x))
    elif delta_x == 0:
        return sign * pi / 2


def find_direction(current_point, goal_point, current_rotation):
    # Use cross product, the right hand rule to determine the direction
    start_vector = (cos(current_rotation), sin(current_rotation))
    end_vector = (goal_point[0] - current_point[0], goal_point[1] - current_point[1])
    if np.cross(start_vector, end_vector) > 0.0:
        return 1
    else:
        return -1


def rotate_towards_goal():
    # Rotate towards the goal
    (position, rotation) = get_odom()
    position_coordinate = (position.x, position.y)
    angle_towards_goal = find_radians_towards_goal(goal, position_coordinate)

    # Loop until rotation is towards the goal
    unsign_rotation = convert_positive_radian(rotation)
    unsign_angle_goal = convert_positive_radian(angle_towards_goal)
    sign = find_direction(position_coordinate, goal, unsign_rotation)

    # find the range of radians that the robot but land on
    upper_bound = unsign_angle_goal + 0.3
    lower_bound = unsign_angle_goal - 0.3

    if upper_bound > 2 * pi:
        upper_bound = upper_bound - 2 * pi
    if lower_bound < 0:
        lower_bound = 2 * pi + lower_bound

    print("Angle Towards Goal:", unsign_angle_goal)
    print("Current Rotation:", unsign_rotation)
    print("Bounds:", upper_bound, lower_bound)
    print("Sign", sign)
    if upper_bound < lower_bound:
        while not (
            (unsign_rotation < upper_bound and unsign_rotation > 0)
            or (unsign_rotation > lower_bound and unsign_rotation < 2 * pi)
        ):
            rotate(sign)
            rotation = get_odom()[1]
            unsign_rotation = convert_positive_radian(rotation)
    else:
        while not (unsign_rotation < upper_bound and unsign_rotation > lower_bound):
            rotate(sign)
            rotation = get_odom()[1]
            unsign_rotation = convert_positive_radian(rotation)

    # Stop the robot before the moving straight
    move_cmd = Twist()
    cmd_vel_pub.publish(move_cmd)
    rospy.sleep(1)


def go_around_left(direction):
    # Initially rotate towards the given direction until there is no object
    print("Going around left side")
    while object_range_ahead <= 1.5:
        rotate(direction)
        r.sleep()

    # Translate two times away from initial position
    translate(1)
    r.sleep()
    translate(1)

    # Report Current distance
    position = get_odom()[0]
    position_coordinate = (position.x, position.y)

    # Loop until the robot is back on x_axis/mline
    while position_coordinate[1] > threshold or position_coordinate[1] < -threshold:
        print("While going around object:", position_coordinate)
        print("Right ahead:", right_ahead)
        print("Range ahead: ", range_ahead)
        print("Left ahead: ", left_ahead)

        # If no object is detected on the right, turn right(optimal)
        if right_ahead > 1.5:
            # turn right until you see an object close
            prev_right = float("inf")
            while right_ahead > 1.5:
                rotate(-direction)
                r.sleep()
                if prev_right < right_ahead:
                    break
                prev_right = right_ahead

            if object_range_ahead > 1.5:
                translate(1)
                position = get_odom()[0]
                position_coordinate = (position.x, position.y)

        # If there is an object on the right, turn left for safety
        elif right_ahead < 1.5:

            while right_ahead < 1.5:
                rotate(direction)
                r.sleep()

            if object_range_ahead > 1.5:
                translate(1)
                position = get_odom()[0]
                position_coordinate = (position.x, position.y)


def go_around_right(direction):
    # Initially rotate towards the given direction until there is no object
    print("Going around right side")
    while object_range_ahead <= 1.5:
        rotate(direction)
        r.sleep()

    # Translate two times away from initial position
    translate(1)
    r.sleep()
    translate(1)

    # Report Current distance
    position = get_odom()[0]
    position_coordinate = (position.x, position.y)

    # Loop until the robot is back on x_axis/mline
    while not (
        position_coordinate[1] < threshold and position_coordinate[1] > -threshold
    ):
        print("While going around object:", position_coordinate)
        print("Right ahead:", right_ahead)
        print("Range ahead: ", range_ahead)
        print("Left ahead: ", left_ahead)

        # If no object is detected on the left, turn left(optimal)
        if left_ahead > 1.5:
            # turn left until you see an object close
            prev_right = float("inf")
            while left_ahead > 1.5:
                rotate(-direction)
                r.sleep()
                if prev_right < left_ahead:
                    break
                prev_right = left_ahead

            if object_range_ahead > 1.5:
                translate(1)
                position = get_odom()[0]
                position_coordinate = (position.x, position.y)

        # If there is an object on the left, turn right for safety
        elif left_ahead < 1.5:

            while left_ahead < 1.5:
                rotate(direction)
                r.sleep()

            if object_range_ahead > 1.5:
                translate(1)
                position = get_odom()[0]
                position_coordinate = (position.x, position.y)


def get_back_ongoal():
    # Rotate towards the goal
    (position, rotation) = get_odom()
    global goal_angle, angular_speed
    global linear_speed, goal_distance
    if position.y > 0.3 or position.y < -0.3 and position.x < 10:
        # Goal is on the right side
        # It is off, so we need to get it back on the m-line
        sign = position.y / abs(position.y)
        print("Sign:", sign)
        # change angular speed

        print("Correcting position back to m-line")
        while not (
            rotation < (-sign * pi / 2 + angular_tolerance)
            and rotation > (-sign * pi / 2 - angular_tolerance)
        ):
            # we want it to face towards the m-line
            rotate(-sign)
            rotation = get_odom()[1]

            if sign == 1.0 and rotation < (-pi / 2 + radians(20)):
                print("Slowing down rotation")
                # slow down to reach precise rotation
                goal_angle = radians(3)
                angular_speed = radians(3)
            if sign == -1.0 and rotation > (pi / 2 - radians(20)):
                print("Slowing down rotation")
                # slow down to reach precise rotation
                goal_angle = radians(3)
                angular_speed = radians(3)

        print("Moving towards mline")
        # then move enough distance to get back on it, it moves too fast so let's change the linear speed

        linear_speed = 0.1
        goal_distance = 0.1
        while not (position.y < 0.1 and position.y > -0.1):
            translate(1)
            position = get_odom()[0]

        # Set it back to the original values
        linear_speed = 0.5
        goal_distance = 0.5
        angular_speed = 1
        goal_angle = radians(15)

        print("Rotating back to the goal.")
        # Rotate towards goal, if the
        while not (rotation < angular_tolerance and rotation > -angular_tolerance):
            # we want it to face towards the m-line
            rotate(sign)
            rotation = get_odom()[1]
            if sign == 1 and rotation > (-radians(20)):
                print("Slowing down rotation")
                # slow down to reach precise rotation
                goal_angle = radians(3)
                angular_speed = radians(3)

            if sign == -1 and rotation < (radians(20)):
                print("Slowing down rotation")
                # slow down to reach precise rotation
                goal_angle = radians(3)
                angular_speed = radians(3)
        angular_speed = 1
        goal_angle = radians(15)
    elif position.y > 0.3 or position.y < -0.3 and position.x > 10:
        # Goal is on the left side
        # It is off, so we need to get it back on the m-line
        sign = position.y / abs(position.y)
        print("Sign:", sign)
        # change angular speed

        print("Correcting position back to m-line")
        while not (
            rotation < (-sign * pi / 2 + angular_tolerance)
            and rotation > (-sign * pi / 2 - angular_tolerance)
        ):
            # we want it to face towards the m-line
            rotate(sign)
            rotation = get_odom()[1]

            if sign == 1.0 and rotation > (-pi / 2 - radians(20)):
                print("Slowing down rotation")
                # slow down to reach precise rotation
                goal_angle = radians(3)
                angular_speed = radians(3)
            if sign == -1.0 and rotation < (pi / 2 + radians(20)):
                print("Slowing down rotation")
                # slow down to reach precise rotation
                goal_angle = radians(3)
                angular_speed = radians(3)

        print("Moving towards mline")
        # then move enough distance to get back on it, it moves too fast so let's change the linear speed
        linear_speed = 0.1
        goal_distance = 0.1
        while not (position.y < 0.1 and position.y > -0.1):
            translate(1)
            position = get_odom()[0]

        # Set it back to the original values
        linear_speed = 0.5
        goal_distance = 0.5
        angular_speed = 1
        goal_angle = radians(15)

        print("Rotating back to the goal.")
        # Rotate towards goal, if the
        while not (
            (rotation > pi - angular_tolerance and rotation < pi)
            or (rotation > -pi and rotation < -pi + angular_tolerance)
        ):
            # we want it to face towards the m-line
            rotate(-sign)
            rotation = get_odom()[1]
            if sign == 1 and rotation < (-pi + radians(20)):
                print("Slowing down rotation")
                # slow down to reach precise rotation
                goal_angle = radians(3)
                angular_speed = radians(3)

            if sign == -1 and rotation > (pi - radians(20)):
                print("Slowing down rotation")
                # slow down to reach precise rotation
                goal_angle = radians(3)
                angular_speed = radians(3)
        angular_speed = 1
        goal_angle = radians(15)


def move_by_iteration(iteration):
    for x in range(iteration):
        translate(1)
        r.sleep
        get_back_ongoal()
        if object_range_ahead < 1.2:
            print("Note safe to move while assuming it was safe")
            break


""" Bugger Navigation Starts """

# Initialize postion, booleans and goal
position = Point()
position_coordinate = (0, 0)
position_before_around = [(float("inf"), float("inf"))]
switch_direction = False
fail = False
rotation = 0
obstacle_encountered = False
goal = (10, 0)

# Translate the robot to reset the odometry
translate(1)
r.sleep()

# Initialize position and rotation with Odometry
(position, rotation) = get_odom()
position_coordinate = (position.x, position.y)
print("Initial position:", position_coordinate)
print("Initial rotation:", rotation)


# Loop until the Robot has reached the goal
while (
    (position_coordinate[0] > goal[0] + threshold)
    or (position_coordinate[0] < goal[0] - threshold)
    or (position_coordinate[1] > goal[1] + threshold)
    or (position_coordinate[1] < goal[1] - threshold)
):

    if obstacle_encountered:
        print("Object is encountered")

        # Clear Move_cmd
        move_cmd = Twist()
        cmd_vel_pub.publish(move_cmd)
        rospy.sleep(1)

        # Initiate direction to go
        if switch_direction == False:
            # right
            go_around_right(-1)
        else:
            # left
            go_around_left(1)

        # Set it to False once it is on the m-line
        print("We are back at the mline!")
        print(position_coordinate[1])
        obstacle_encountered = False

    else:
        rotate_towards_goal()
        print("Object not encountered:", object_range_ahead)
        (position, rotation) = get_odom()
        # Move straight to the goal or to the safest distance
        if object_range_ahead > 1:
            safe_distance_to_travel = object_range_ahead - 1
            distance_to_goal = sqrt(
                pow((position.x - goal[0]), 2) + pow((position.y - goal[1]), 2)
            )

            if distance_to_goal <= safe_distance_to_travel:
                # Safe to go straight for the goali
                print("Safe|Going to Goal!")
                iteration = int(math.floor(distance_to_goal / goal_distance))
                move_by_iteration(iteration)

            else:
                # Not safe, we can only move what is safe enough
                iteration = int(math.floor(safe_distance_to_travel / goal_distance))

                # Iteration is zero when the robot is 1m away from object, can't move at all
                if iteration == 0:
                    # Goal is within 'unsafe' travel zone
                    if distance_to_goal < object_range_ahead:
                        # safe to move towards goali
                        print("Goal is within unsafe range")
                        iteration = int(math.floor(distance_to_goal / goal_distance))
                        move_by_iteration(iteration)
                    else:
                        print("Not safe at all")
                        """ Has to be the same code if obstacle is encountered """
                        # Before we switch record the position
                        position = get_odom()[0]
                        position_coordinate = (position.x, position.y)

                        match_position = False
                        for obs in position_before_around:
                            if (
                                position_coordinate[0] <= obs[0] + 1
                                and position_coordinate[0] >= obs[0] - 1
                            ):
                                if (
                                    position_coordinate[1] <= obs[1] + 1
                                    and position_coordinate[1] >= obs[1] - 1
                                ):
                                    # we are back at where we were before
                                    print("Same position as last time!")
                                    print("Position before going around:", obs)
                                    print("Current position:", position_coordinate)
                                    match_position = True
                                    break

                        if match_position == False:
                            print(
                                "Position before going around:", position_before_around
                            )
                            print("Current position:", position_coordinate)
                            position_before_around.append(position_coordinate)
                            if switch_direction == True:
                                # reset it
                                switch_direction = False

                        if match_position and switch_direction != True:
                            # If the position is the same as last time, switch direction
                            switch_direction = True
                        elif match_position and switch_direction == True:
                            # If we have already tried the other direction, then we need to exit
                            fail = True
                            break

                        # Set to true anyways to go around the other object
                        obstacle_encountered = True
                else:
                    print("Only moving to safe distance")
                    move_by_iteration(iteration)

            # Update position and rotation before the iteration ends
            position = get_odom()[0]
            position_coordinate = (position.x, position.y)
            print("Position after towards goal", position_coordinate)

        else:
            # Before we switch record the position
            position = get_odom()[0]
            position_coordinate = (position.x, position.y)

            match_position = False
            for obs in position_before_around:
                if (
                    position_coordinate[0] <= obs[0] + 1
                    and position_coordinate[0] >= obs[0] - 1
                ):
                    if (
                        position_coordinate[1] <= obs[1] + 1
                        and position_coordinate[1] >= obs[1] - 1
                    ):
                        # we are back at where we were before
                        print("Same position as last time!")
                        print("Position before going around:", obs)
                        print("Current position:", position_coordinate)
                        match_position = True
                        break

            if match_position == False:
                print("Position before going around:", position_before_around)
                print("Current position:", position_coordinate)
                position_before_around.append(position_coordinate)
                if switch_direction == True:
                    # reset it
                    switch_direction = False

            if match_position and switch_direction != True:
                # If the position is the same as last time, switch direction
                switch_direction = True
            elif match_position and switch_direction == True:
                # If we have already tried the other direction, then we need to exit
                fail = True
                break

            # Set to true anyways to go around the other object
            obstacle_encountered = True

if fail:
    print("Failed to reach the goal! We tried both direction, there is no way out!")
    print(position_coordinate)
else:
    print("We reached the goal!")
    print(position_coordinate)
cmd_vel_pub.publish(Twist())
