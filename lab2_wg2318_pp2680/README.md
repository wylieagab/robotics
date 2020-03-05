# Instructions for Lab 2
1. Launch bug world. There are six worlds in total, including extra credit.
```
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$PWD/bug2_0.world
```
2. Run the bug algorithm.
```
./bug2.py cmd_vel:=cmd_vel_mux/input/teleop
```

# Methods Used in Bug Algorithm
scan_callback(msg) - Updates distances of obstacle to robot from left side, right side, directly in front, and overall closest to the robot

get_odom() - Returns the current position and sign rotation of the robot relative to the world. 

translate(sign) - Given the direction/speed/goal_distance, it moves the robot in that direction and speed up to the goal distance in one cycle. If negative, backwards. If positive, forward. 

rotate(sign) - Given the direction/speed/angular_goal, it rotates the robot in that direction and speed up to the angular goal in one cycle. If negative, clockwise. If positive, counter-clockwise.

convert_positive_radian(sign_radian) - Given the sign radian, convert it to a positve radian. Since the odometry returns values between -pi to pi, this function returns values between 0 to 2pi. This was used to get an upper and lower bound when rotating towards the goal. Makes computation a lot easier. 

find_radians_towards_goal(goal, position_coordinate) - Given coordinates of goal and current position, it returns sign rotation that will have the robot turns toward the goal. 

find_direction(current_point, goal_point, current_rotation) - Given coordinates of goal and current position and its current rotation, it returns the best direction to rotate by computing right hand rule for efficiency.  

rotate_towards_goal() - This function called only once when obstacle is no longer encountered. Using two functions above, it rotates the robot towards the goal with optimal rotation and direction.

go_around_left(direction) - Given the direction which is left, it initially rotates the robot left then moves around obstacle by keep checking for an obstacle on the right side. This function ends once it reaches the m-line.

go_around_right(direction) - Given the direction which is right, it initially rotates the robot right then moves around the obstacle by keep checking for an obstacle on the left side. This function ends once it reaches the m-line

get_back_ongoal() - This function is always called when the robot is not encountering an obstacle and just wants to move towards the goal by the safest distance. If the robot is off from the m-line, this function will correct its position back to the m-line. It rotates towards the m-line, moves towards it, rotate back to the direction to the goal. For precision, the rotation will slow down. 

move_by_iteration(iteration) - Given the number of iterations, the robot moves forward by the number of iterations. Each time it moves forward, it checks if it off the m-line, if so then call get_back_ongoal().

# Bug Algorithm Pseudo Code
This bug algorithm is a bit different from the one that the Professor taught. 

For the sake of time to prove that the extra credit world works, the robot will turn right and around the obstacle if it is the first time encountered that obstacle at a different position. If the robot notices that it encounters an obstacle again from the same position last time, then the robot needs to try an alternative path which is turning left and around the obstacle. If the robot encounters an obstacle and it is the same positon as last time we encountered an obstacle and the robot has already tried both direction, then it is correct to say that there is no solution/path to the goal. 

Robot will always resort to right turn and around the obstacle until it reaches the m-line. If that path brings the robot to the same position as before, we have try a different path which is turn left and around the obstacle. If both direction doesn't work, then we can say it failed. It doesn't matter if we turn left as our first path, as long as we tried turning right if the robot comes back to the same position. 

The reason we chose to turn right first is because turning right first in the extra credit world saves more time to reach the goal. If we chose to turn left first, the algorithm still works because when the robot enters the maze, it goes around the maze back to where it started. To solve this problem, there is a list of obstacles that robot have encountered. We keep track of this list so that we can tell the robot to turn right if it has already tried turning left. There might be some edge cases that won't be satisfied because there is one boolean that sets the whole robot to turn left or right. There has to be a list of booleans corresponding to each obstacle it has visited. For now, it will switch direction if any of the obstacle has been reached.  

This might be hard to explain in words. A diagram is shown in a pdf file attached in this folder. 

```
While loop until position is near the goal:
  If an obstacle is encountered:
    If this is our first try going around an object:
      Rotate Right and Move Forward
      Loop until we reach the m-line:
        Rotate Left Until it is safe to move 
        Move forward
    If this is our second try going around an object:
      Rotate Left and Move Forward
      Loop until we reach the m-line:
        Rotate Right until it is safe to move
        Move forward
    Reached the m-line, obstacle is no longer encountered
  Else If obstacle is not encountered:
    Rotate towards the goal
    Calculate the safest distance to move
    Move forward and constantly correct back to the m-line
    if the position is the same as the previous[is in the list of obstacles we have visited] and we have not switched:
      change the direction to go around the object
    else if we have switched and same position again:
      terminate the loop and report failure to find path
    else we have never came across this obstacle:
      add to list of obstacles we have visited
```
# Youtube Video
https://www.youtube.com/watch?v=AdlXBAVBk6A&list=PLfprnTCr2zWWVcIUSOqTvj-uLmcy7dZeS

Side Notes: 

Bug is always keeping its distance >1 from obstacles.

It is very slow because I want it precisely on the m-line so that world 5 is a fail. I zoomed in some parts so you can notice that the robot is correcting its position, it's not frozen. 

Two grace days used.

Extra credit attempted and succeeded

There are two videos because my recorder stops recording at 2 hours so the second video is just 3 minutes left over and you can check the time on the top right to see that it wasn't a separate recording.
