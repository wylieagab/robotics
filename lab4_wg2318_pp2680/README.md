# Lab 4 - Rapidly-exploring Random Tree (RRT)

# Instructions
There are no other scripts other than demo.py. Just run those commands in your terminal. No external files added.

demo.py is a script that uses RRT algorithm to find a path for the robot arm to travel from start to goal. There are three joints with limitations and 3 obstacles which are two boxes and the floor. 

RRT uses random sampling to connect closely to the existing Tree with 5% bias towards the goal. It is not the most efficient path to the goal because it stops immediately once we have reached there.

Bidirectional RRT is quite similar but instead of one tree, there are two trees. Both trees alternate in extending its graph towards their own target and each other. The goal of this algorithm is to have them connect at some point. Once connected, we terminate once we have found a path. 

With smoothing, we can trim down some edges and avoid travelling further than it needs to. Aslong as the path is collision-free and reaches the goal. The remaining points are very minimal so linear interpolation is needed to create a smoother/curvier path for the arm. 

Part 1 - RRT
```
python demo.py
```
Part 2 - Bidirectional RRT
```
python demo.py --birrt
```
Extra Credit - Path Smoothing for Bidirectional RRT
```
python demo.py --birrt --smoothing
```
Demo for all 3 parts: https://www.youtube.com/watch?v=7aqwjJsVGQI&feature=youtu.be

# Methods 
set_joint_positions(body, joints, values) - sets the robot arm to given values

draw_sphere_marker(position, radius, color) - draw dots in 3D simulation

remove_marker(marker_id) - removes markers that were made in the simulation

get_args() - calls the right path finding function

getEndEffectorState(conf) - moves the robot arm to the given configuration and returns the position of the head of the arm

Tree() - class used to create Tree objects with vertices, edges and N which is the maximum number of configuration attempts

genConfRandom(min_val, max_val) - returns a random value between min and max

distance_euclid(p1, p2) - returns the euclidean distance between two points

nearest_n(vertices, q_rand) - finds the nearest node to the randomize configuration

progress(q_near, dest_conf, step = step_size) - progresses a new configuration given the nearest neighbor and randomize configuration

close_to_target(q, target) - checks whether the configuration is within the step_size of the target

update_tree(T, existing_node, new_node, color) - connects the new node to the existing node from the Tree and draws a new edge in the simulation with the given color

extend_rrt(T, q_rand, goal, color="green") - given the randomize configuration, this creates a new configuration that is a step_size away from neighbor and connects it to the tree if it is collision-free, everytime a new node is added to the tree, it checks whether it is within the range of the target/goal

random_config(target) - randomize configuration that is 5% biased towards the target

build_RRT() - creates a Tree and extends it with randmize/biased/collision-free nodes until it reaches the goal

DFS(T, start, end) - Depth First Search algorithm used to find a path of a Tree from start to end

rrt() - grows one tree until a path is found

build_biRRT() - creates two trees and alternate growth towards each other until they are both connected, uses similar implementation as RRT but different targets

birrt() - finds a path in T1 from start to connecting node, finds a path in T2 from connecting node to goal, then returns the concatenated paths of two trees with DFS

check_collision(first, second) - given two points, checks for collision for each segment of the line 

birrt_smoothing() - just like BiRRT but added smoothing algorithm, iterate N times and randomize two points in the original path, snip the path so that connection between two points is collision-free, with those remaining points interpolate points in between to create a more curvier and smoother path 




