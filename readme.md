# 3806ICT assignment 3

- Joshua Gilbert (s5233877)
- Ojaswini Sharma (s5306997)
- Harvey Shaw (s5277717)

This project requires a Ubuntu 20.04 environment with ROS, Gazebo and Python 3 installed. To successfuly run the project on your own machine you must:

1. Clone this repository into a catkin workspace's source directory (/catkin_ws/src).
2. Run catkin_make from the root of the catkin workspace (/catkin_ws).
3. Launch the gazebo launch file using: roslaunch 3806Robotics launch_world.launch
4. In a new terminal window run the FRL training and model simulation using: rosrun 3806Robotics RunSimulation.py /path-to-catkin-ws

Note: the path provided to the command line when running the python script must begin with a '/' and not end with a '/' otherwise the program will not be able to find the model files for the gazebo simulation. For example if the catkin workspace was located in the home directory of a user named student the path would be: /home/student