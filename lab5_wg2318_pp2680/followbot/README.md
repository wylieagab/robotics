# Lab 5 - Follow Bot

## Usage
This ROS package allows you to load 4 different maps in Gazebo.

### Prerequisites
The package is tested on `python 2.7`, `ROS Indigo`, `Ubuntu 14.04` with `OpenCV 3.1.0` and `numpy 1.15.1`.

### Commands
To launch turtlebot and map for [part 1](#part-1-preparation-20-points)[Complete]
```
roslaunch followbot launch.launch
python follower_color.py
```

To launch turtlebot and map for [part 2](#part-2-map-with-color-markers-40-points)[Complete]
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=color.world
python follower_shape.py
```

To launch turtlebot and map for [part 3](#part-3-map-with-shape-markers-40-points)[Incomplete]
* Was able to follow the yellow line and determine direction based on vertices but was not able to stop at the star. 
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=shape.world
python follower_shape.py
```
There are two scripts in the submission.
follower_color.py is used for part 1 and part 2 to detect colors.
follower_shape.py is used for part 3 to detect shapes.
Both scripts are based on Follower class from the textbook. All color and shape detection implementation are from OpenCV. 
### Methods from both scripts

* imageCallback(self, msg) - reads in the messages from the camera and determines to stop or follow left or right based on shape or color
* detectShape(self, msg) - return the detected color 
* detectColor(self, msg) - return the detected shape and direction 
* blobsAboveShape(self, shape_centroid, hsv) - return yellow moments above detected color, number of vertices determine shape 
* blobsAboveColor(self, color_centroid, hsv) - return yellow moments above detected shape 
* blobBelow(self, x, y, hsv) - return whether or not there is a yellow moment below the shape to avoid weird vertices in the conditional
* move(self, cx) - moves the robot towards the center of yellow
* getYellowMask(self, hsv, threshold) - returns a yellow mask of an hsv image
* followBlob(self, direction, blobs_list, image) - given that there are two moments above color/shape, we will follow left or right within intersection
* followYellow(self, hsv, image) - follow yellow line when not in intersection

### Video Demonstration 

Part 1:https://youtu.be/mzpJ4FHKZX4
Part 2:https://youtu.be/hTq4vvln4Hk



