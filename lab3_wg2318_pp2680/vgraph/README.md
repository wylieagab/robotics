# Lab 3 - Visibility Graph Path Planning Group 7
## Instructions 
1. Launch the map.
```
roslaunch vgraph launch.launch
```
2. Start the script at vgraph/src/ directory 
```
 ./turtlebot_nav.py visualization_marker_array:=vgraph_markers_array
```
Demo: https://www.youtube.com/watch?v=3sSqv2tFPUk&feature=youtu.be
## Description
This script will observe all of the obstacles in the world and robot's location. It will create a convex hull around the obstacles and find all possible edges from node to node without colliding other obstacles. Then using the Dijsktra'a algorithm, it will find the shortest path for the robot to travel to the goal. Then it will move the robot from functionalities used from Lab 1 and Lab 2. This is all in one script, no need to run other programs. 

## Methods 
load_obstacles() - reads 'world_obstacles.txt' and store all vertices of given obstacles

grow_obstacle_shell() - use convex hull to grow the obstacles 

create_line(point1, point2, color="red") - draw a line given two points and color, just red or green for now

find_obstacle_edges() - store information about all of the existing edges around obstacles

draw_connected_graph() - given the growing obstacles and start point and goal point, create and draw new edges that don't collide with other obstacles

collision_check(start_point_A, end_point_A) - checks if the new edge collides with any of the obstacle edges

calculate_distance(pointA, pointB) - returns the euclidean distance between two points

find_shortest_path() - uses Dijkstra's algorithm to find the shortest path of visited nodes until the goal is reached, returns the sequence of nodes to travel to 

move(path) - given the path, uses functionalies of Lab 1 and Lab 2 to move the robot from point to point

## From Lab 2 - Check lab 2 for description, not much changed 
get_odom()

translate(sign)

rotate(sign)

find_direction(current_point, goal_point, current_rotation)

convert_positive_radian(sign_radian)

find_radians_towards_dest(goal, position_coordinate)
