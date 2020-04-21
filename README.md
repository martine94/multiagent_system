# multiagent_system
Catkin repository for multiagent_system course


Prerequisite:

To run this project you need to have Turtlebot3 in your catkin workspace.
You also need to have gazebo installed.


After building:


To run, first start the map. This is done from the launch folder by entering the following command:

$ roslaunch multiagent.launch

If everything has been installed correctly, you should have two very simple maps/worlds that you can use in gazebo.
Input one or multiple turtle-bots within this map. At the moment, the project only moves one turtle-bot forward.
The goal at the moment is to implement an avoid and random-walk function for multiple turtlebots.



To then run the project, use the following command:

$ rosrun multiagent_system multiagent_system
