For first time setup

cd turtlebot3_ws

colcon build

source ~/turtlebot3_ws/install/setup.bash

To run with teleop do this:

ros2 launch cde2310_g4_ay2526 custom_maze.launch.py

This should get you rviz + gazebo with the custom maze and rviz is populated by the occupancy graph
