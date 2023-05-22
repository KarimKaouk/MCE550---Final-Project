Insert the "final_project" package into the src file in your ROS Workspace.

Assuming the file "Assignment 1" is your Workspace,
Run the following commands in seperate terminals: 
(In order to open the terminals use CTRL + SHIFT + E/O )


**************************************************************************************
Setup package in your workspace:
------------------------------- 

Terminal 1:
----------

>> cd ~/catkin_ws
>> catkin_make
>> source devel/setup.bash
>> roscore

**************************************************************************************
Run the simulation in "simworld" using "turtlebot3_waffle":
---------------------------------------------------------
To ensure proper running , copy the "simworld" file from "catkin_ws/src/final_project/worlds" and paste it in 
"catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds".
Then delete the "empty.world" and rename the "simworld" to "empty.world".
This ensures the turtlebot3 waffle is properly spawned with all its services and plugins functional.

Terminal 2:
----------

>> roslaunch final_project testfinal.launch


After launching this, in the same terminal input the (x , y) coordinates of the goal in the following format:
x y 

This sets your goal to these coordinates if the format was valid and the values numerical.

*******************************************
View any of the 2 worlds "simworld" and "final_world": 
----------------------------------------------------
Terminal 3:

>> cd ~/catkin_ws/src/final_project/worlds
>> gazebo simworld
       (or)
>> gazebo final_world

***************************************************************************************
