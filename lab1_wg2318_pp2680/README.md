# Instructions for Lab 1 
1. Launch the robot file 
```
roslaunch rbx1_bringup fake_turtlebot.launch 
```
2. Start the simulator with the robot. Linux back tick operator did not work for me. 
So I simply called the simulator located in my system.
```
rosrun rviz rviz -d /home/phivian/catkin_ws/src/rbx1/rbx1_nav/simr.viz
```
3. Once rviz is loaded, go ahead and run the script that controls the robot.
```
rosrun rbx1_nav timed_out_and_back.py
```
You will be prompt to enter 'T' for translation, 'R' for rotation, or 'Q' to quit.
If 'T', you will be prompt to enter the number of meters to travel. Which means you can also travel forward 
with positive values and backwards with negative values. It can accept any real numbers. 
If 'R', you will be prompt to enter degrees to rotate. Which means you can also rotate counter-clockwise 
with positive degrees and clockwise with negative degrees. It can accept any real numbers. Keep in mind 
that the robot will just be spinning in circles if less than -360 or greater than 360. 
You can only do one operation at a time. The prompt will only show up once the operation is completed. 

Here is our demo: https://www.youtube.com/watch?v=ouwDnzGg5Zc

Complications: I noticed that the accuracy of this robot is very inconsistent. There were times that it was completely
off and times it was on point. I noticed that the only factor that caused this inconsistency is having other features 
run on Virtual Box. For example, whenever I tried to record the demonstration with VirtualBox it would slow down the 
Rosrate which caused the robot to be 'faster' and travelled more than it should. In the future, I need to have Ubuntu 
run efficiently without affecting the Rosrate. And prevent internal functionalities in the future. 
