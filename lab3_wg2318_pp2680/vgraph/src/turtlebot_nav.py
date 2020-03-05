#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi, atan, cos, sin
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
import numpy as np
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import collections


def load_obstacles():
    # Instantiate obstacles array when in /catkin_ws/src
    f = open("../data/world_obstacles.txt")
    counter = -1
    array_len = int(f.readline())
    obstacles = [0] * array_len
    obstacles = [[] for x in range(array_len)]
    for line in f:
        entry = line.split()
        if len(entry) == 1:
            counter += 1
        else:
            obstacles[counter].append([int(entry[0]), int(entry[1])])

    # print ("Loaded obstacles: ", obstacles)
    return obstacles


# Initialize node "turtle_nav":
rospy.init_node("turtle_nav", anonymous=False)

# Publisher to control robot movement
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

# Publisher to make convexhull
vis_marker_pub = rospy.Publisher(
    "visualization_marker_array", MarkerArray, queue_size=1
)

# How fast will we upadte the robots movement
rate = 20

# Set ROS rate variable equivalent
r = rospy.Rate(rate)

# Set linear speed
linear_speed = 0.5

# Singular travel distance in meters
goal_distance = 0.1

# Linear Threshold
threshold = 0.1

# Angular rotation in rads/s
angular_speed = radians(1)

# Angular tolerance at 2.5 degrees
angular_tolerance = radians(3)

# Maximum single rotation angle
goal_angle = radians(4)

# Initialize the tf listener
tf_listener = tf.TransformListener()

# Give tf time to fill buffer
rospy.sleep(2)

# Set the odom fram
odom_frame = "/odom"

# Vertices of Obstacles
obstacles = load_obstacles()

# Array containing set of obstacle shells
marker_array = MarkerArray()

# List of lists containing hull vertex coordinates
obstacle_hull_array = dict()

# List of obstacle edges
obstacle_edges = list()

# Nodes
graph_nodes = dict()

# Edges
graph_edges = dict()

# Distances will be calculated later

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


def grow_obstacle_shell():
    """
    Given the obstacles, use ConvexHull to grow the obstacles. 
    Draws the boundary around each obstacle.
    """
    # Array containing set of obstacle shells
    global marker_array

    for obstacle in obstacles:

        # Marker Instantiation
        marker = Marker()
        marker.header.frame_id = "/base_link"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # marker orientation
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Array of 2D Pts For Obstacle Shell
        marker.points = []

        # Candidate Array for Convex Hull
        convex_hull_vertices = list()

        # Candidate Instantiation
        for vertex in obstacle:
            convex_hull_vertices.append([(vertex[0] + 18) / 100.00, vertex[1] / 100.00])
            convex_hull_vertices.append([(vertex[0] - 18) / 100.00, vertex[1] / 100.00])
            convex_hull_vertices.append(
                [(vertex[0]) / 100.00, (vertex[1] + 18) / 100.00]
            )
            convex_hull_vertices.append(
                [(vertex[0]) / 100.00, (vertex[1] - 18) / 100.00]
            )
            convex_hull_vertices.append(
                [(vertex[0] + 18) / 100.00, (vertex[1] + 18) / 100.00]
            )
            convex_hull_vertices.append(
                [(vertex[0] - 18) / 100.00, (vertex[1] + 18) / 100.00]
            )
            convex_hull_vertices.append(
                [(vertex[0] - 18) / 100.00, (vertex[1] - 18) / 100.00]
            )
            convex_hull_vertices.append(
                [(vertex[0] + 18) / 100.00, (vertex[1] - 18) / 100.00]
            )

        # Generate Convex Hull
        hull = ConvexHull(convex_hull_vertices)
        # print(hull.equations)

        # Grab the vertices returned by convexhull
        marker_vertice_array = list()
        for x in hull.vertices:
            convex_hull_vertices[x].append(1)
            marker_vertice_array.append(convex_hull_vertices[x])

        # Add obstacle hull coordinates to its container
        global obstacle_hull_array
        obstacle_hull_array.update({len(obstacle_hull_array) + 1: marker_vertice_array})

        # Instantiate Marker and add it to the marker array (Construct Obstacle Shell)
        for point in marker_vertice_array:
            line_point = Point()
            line_point.x = point[0]
            line_point.y = point[1]
            line_point.z = 0.0
            marker.points.append(line_point)

        # Add first point to complete the outline
        line_point = Point()
        line_point.x = marker_vertice_array[0][0]
        line_point.y = marker_vertice_array[0][1]
        line_point.z = 0.0
        marker.points.append(line_point)

        # Add generated obstacle shell to the marker_array
        marker_array.markers.append(marker)

    # Assign Ordered Id
    m_id = 0
    for m in marker_array.markers:
        m.id = m_id
        m_id += 1

    # Publish Obstacle Shells
    vis_marker_pub.publish(marker_array)

    # Output function exit and sleep
    print("Obstacle Shells Instantiated...")
    rospy.sleep(1)


def create_line(point1, point2, color="red"):
    """
    Given two points, returns a Marker of a line segment. 
    """
    # Marker Instantiation
    marker = Marker()
    marker.header.frame_id = "/base_link"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD

    if color == "red":
        # marker scale
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    else:
        # marker scale
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        # marker color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

    # marker orientation
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # Array of 2D Pts For Obstacle Shell
    marker.points = []

    # Generate the connected set of points
    line_point = Point()
    line_point.x = point1[0]
    line_point.y = point1[1]
    line_point.z = 0.0
    marker.points.append(line_point)

    line_point = Point()
    line_point.x = point2[0]
    line_point.y = point2[1]
    line_point.z = 0.0
    marker.points.append(line_point)

    return marker


def find_obstacle_edges():
    global obstacle_edges
    for obstacle_points in obstacle_hull_array.values():
        for x in range(len(obstacle_points)):
            if x == (len(obstacle_points) - 1):
                obstacle_edges.append([obstacle_points[x], obstacle_points[0]])
            else:
                obstacle_edges.append([obstacle_points[x], obstacle_points[x + 1]])


def draw_connected_graph():
    """
    For each point generated from growing obstacles, connect point to every other point 
    in the map including the starting point and the goal. 
    """
    print("Connecting Vertices....")
    # Instantiate MarkerArray for filling and later publishing
    global marker_array, graph_edges, graph_nodes

    # Add in start and goal points
    start = [0, 0, 1]
    graph_nodes.update({0: start})
    graph_edges.update({0: []})

    f = open("../data/goal.txt")
    goal = f.readline().split()
    goal = [int(x) / 100 for x in goal]
    goal.append(1)
    graph_nodes.update({1: goal})
    graph_edges.update({1: []})

    # Loop through every point in the Obstacle
    for obstacle_key, obstacle_points in obstacle_hull_array.items():
        for point in obstacle_points:
            # Create a new node for this node
            current_index = len(graph_nodes)
            graph_nodes.update({current_index: point})

            # Connect this point to every other points that don't belong in this obstacle
            for other_key, other_points in obstacle_hull_array.items():
                if other_key != obstacle_key:
                    for other_point in other_points:
                        # check if there are any collision with other edges
                        if collision_check(point, other_point):
                            # create a new edge
                            if current_index not in graph_edges.keys():
                                graph_edges.update({current_index: [other_point]})
                            else:
                                graph_edges[current_index].append(other_point)
                            # Create a Line Marker
                            marker = create_line(point, other_point)
                            # Add them to the marker_array
                            marker_array.markers.append(marker)

            # Connect to start position and goal

            if collision_check(point, start):
                # Add to current node
                if current_index not in graph_edges.keys():
                    graph_edges.update({current_index: [start]})
                else:
                    graph_edges[current_index].append(start)

                # Add to start node
                graph_edges[0].append(point)

                # Create a Line Marker
                marker = create_line(point, start)
                # Add them to the marker_array
                marker_array.markers.append(marker)

            if collision_check(point, goal):
                # Add to current node
                if current_index not in graph_edges.keys():
                    graph_edges.update({current_index: [goal]})
                else:
                    graph_edges[current_index].append(goal)

                # Add to goal node
                graph_edges[1].append(point)

                # Create a Line Marker
                marker = create_line(point, goal)
                # Add them to the marker_array
                marker_array.markers.append(marker)

    # Don't forget obstacle edges
    for edge in obstacle_edges:

        # Find the index of the first point
        first_index = list(graph_nodes.values()).index(edge[0])
        print(first_index)
        graph_edges[first_index].append(edge[1])

        # Find the index of the second point
        second_index = list(graph_nodes.values()).index(edge[1])
        graph_edges[second_index].append(edge[0])

    # Assign Ordered Id
    m_id = 0
    for m in marker_array.markers:
        m.id = m_id
        m_id += 1

    # Publish Marker Array
    vis_marker_pub.publish(marker_array)


def collision_check(start_point_A, end_point_A):
    """
    Use numpy(dot product) to find where these two line segment intersect. 
    If the intersection is not at the end points, then there is a collision.
    """
    print("Checking collision...")
    # Get equation of line from given point
    line = np.cross(start_point_A, end_point_A)

    for edge in obstacle_edges:
        # Get equation of line from given points of current edge
        test_edge = np.cross(edge[0], edge[1])
        list_all_points = [start_point_A, end_point_A, edge[0], edge[1]]

        # Use Cross Product to find the point of intersection between two line segments
        point_of_intersection = list(np.cross(line, test_edge))
        print(list_all_points)
        print(point_of_intersection)
        # Check if Z is equal to 0
        if point_of_intersection[2] != 0.0:
            point_of_intersection = [
                round(point_of_intersection[0] / point_of_intersection[2], 2),
                round(point_of_intersection[1] / point_of_intersection[2], 2),
            ]
            print(point_of_intersection)

            is_vert = False
            # Check if the intersection is one of the given points
            count = 0
            for vert in list_all_points:
                if collections.Counter(vert[:2]) == collections.Counter(
                    point_of_intersection
                ):
                    # Need two
                    count += 1
            print(count)
            if count == 2:
                is_vert = True
            # Check if it inside both projections
            in_proj = False
            x = point_of_intersection[0]
            y = point_of_intersection[1]
            first_lower_x = min(start_point_A[0], end_point_A[0])
            first_lower_y = min(start_point_A[1], end_point_A[1])
            first_higher_x = max(start_point_A[0], end_point_A[0])
            first_higher_y = max(start_point_A[1], end_point_A[1])
            second_lower_x = min(edge[0][0], edge[1][0])
            second_lower_y = min(edge[0][1], edge[1][1])
            second_higher_x = max(edge[0][0], edge[1][0])
            second_higher_y = max(edge[0][1], edge[1][1])

            if (
                (x >= first_lower_x and x <= first_higher_x)
                and (y >= first_lower_y and y <= first_higher_y)
                and (x >= second_lower_x and x <= second_higher_x)
                and (y >= second_lower_y and y <= second_higher_y)
            ):
                in_proj = True
            # Found intersection, it is a fail
            if is_vert == False and in_proj == True:
                print("Found collision, Next")
                return False

    # All of the edges pass
    print("No Collision")
    return True


def calculate_distance(pointA, pointB):
    return sqrt((pointA[0] - pointB[0]) ** 2 + (pointA[1] - pointB[1]) ** 2)


def find_shortest_path():
    print(graph_nodes)
    print(graph_edges)

    # Initialize all nodes to be infinite
    nodes_weights = dict()
    prev_nodes = dict()
    for x in graph_nodes.keys():
        if x == 0:
            nodes_weights.update({0: 0})
        else:
            nodes_weights.update({x: float("inf")})
        prev_nodes.update({x: np.nan})

    visited_nodes = [0]  # has indexes, not actual points

    while collections.Counter(visited_nodes) != collections.Counter(graph_nodes.keys()):

        # Find the node with the shortest path
        min_distance = float("inf")
        min_node = float("inf")
        prev_node = float("inf")

        # list of unvisited nodes that have edges with visited nodes
        for node_key, node_value in graph_nodes.items():
            if node_key not in visited_nodes:
                # Check if this has an edge with a visited node
                for edge_point in graph_edges[node_key]:
                    for v in visited_nodes:
                        if collections.Counter(edge_point) == collections.Counter(
                            graph_nodes[v]
                        ):
                            # Calculate the distance
                            distance = nodes_weights[v] + calculate_distance(
                                node_value, edge_point
                            )
                            if distance < min_distance:
                                min_distance = distance
                                min_node = node_key
                                prev_node = v

        # Update the node with the min distance
        visited_nodes.append(min_node)
        nodes_weights[min_node] = min_distance
        prev_nodes[min_node] = prev_node

    print(prev_nodes)
    # Print path
    n = 1
    path = [n]
    while n != 0:
        path.append(prev_nodes[n])
        n = prev_nodes[n]
    path.reverse()

    print(path)
    global marker_array
    for p in range(len(path) - 1):
        marker = create_line(graph_nodes[path[p]], graph_nodes[path[p + 1]], "green")
        marker_array.markers.append(marker)

    # Assign Ordered Id
    m_id = 0
    for m in marker_array.markers:
        m.id = m_id
        m_id += 1

    # Publish Marker Array
    vis_marker_pub.publish(marker_array)
    return path


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
    print("Rotating robot...:", convert_positive_radian(rotation))


def find_direction(current_point, goal_point, current_rotation):
    # Use cross product, the right hand rule to determine the direction
    start_vector = (cos(current_rotation), sin(current_rotation))
    end_vector = (goal_point[0] - current_point[0], goal_point[1] - current_point[1])
    if np.cross(start_vector, end_vector) > 0.0:
        return 1
    else:
        return -1


def convert_positive_radian(sign_radian):
    # convert all radians to be positive
    if sign_radian < 0:
        return 2 * pi + sign_radian
    else:
        return sign_radian


def find_radians_towards_dest(goal, position_coordinate):
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


def move(path):
    path = path[1:]
    for step in path:
        position, rotation = get_odom()
        position_coordinate = (position.x, position.y)
        destination = (graph_nodes[step][0], graph_nodes[step][1])
        angle_towards_goal = find_radians_towards_dest(destination, position_coordinate)

        unsign_rotation = convert_positive_radian(rotation)
        unsign_angle_goal = convert_positive_radian(angle_towards_goal)
        sign = find_direction(position_coordinate, destination, unsign_rotation)

        # find the range of radians that the robot but land on
        upper_bound = unsign_angle_goal + angular_tolerance
        lower_bound = unsign_angle_goal - angular_tolerance

        print("Current Rotation:", unsign_rotation)
        print(upper_bound, lower_bound)
        if upper_bound > 2 * pi:
            upper_bound = upper_bound - 2 * pi
        if lower_bound < 0:
            lower_bound = 2 * pi + lower_bound

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

        while not (
            (
                position_coordinate[0] < destination[0] + threshold
                and position_coordinate[0] > destination[0] - threshold
            )
            and (
                position_coordinate[1] < destination[1] + threshold
                and position_coordinate[1] > destination[1] - threshold
            )
        ):
            translate(1)
            position = get_odom()[0]
            position_coordinate = (position.x, position.y)

        # Stop robot completely
        move_cmd = Twist()
        cmd_vel_pub.publish(move_cmd)
        rospy.sleep(1)


if __name__ == "__main__":
    grow_obstacle_shell()
    print("Grown Obstacle Points")
    print(obstacle_hull_array)
    find_obstacle_edges()
    print(obstacle_edges)
    print("------------------------------------------------------")
    draw_connected_graph()
    print("------------------------------------------------------")
    path = find_shortest_path()
    move(path)
