# flipkart_d2c

# Autonomous Indoor Drone (Team UMIC Aerial)

This is the Github Repository of Team UMIC Aerial as a part of its presentation for the Autonomous Indoor Drone Challenge for the Flipkart GRiD 2.0 Finals. 


## Installation

To install this setup on your system, you need Ubuntu 18.04 with ROS Melodic as your OS. 

Install ROS Melodic using the following link:
http://wiki.ros.org/melodic/Installation/Ubuntu 

For the Simulation and Control of the whole mission, we are using the PX4 SITL (Software in the loop) Simulator, along with the MAVROS protocol. 
To Install the full setup on your system, follow the steps in the link given below:
https://dev.px4.io/master/en/setup/getting_started.html
https://dev.px4.io/master/en/simulation/ros_interface.html

For the Autopilot Path Planning code, we use the PX4 stack, along with Python. You should already have that installed at this point. 

For the Localization module, we are using ORBSLAM2. Installation instructions can be found below:
https://github.com/appliedAI-Initiative/orb_slam_2_ros

For the Frame Detection module, we are using the OpenCV Library. To install that, follow the following link:
 https://docs.opencv.org/master/d2/de6/tutorial_py_setup_in_ubuntu.html


This should setup the required software in your system. After this, clone our repository in your workspace, and build it. If the code compiles without any errors, you are good to go. Source your workspace and proceed ahead :)
If there are any errors while building the code, go through your installation again and check if you missed out on anything. 

## Structure Explanation

We have the following structure to our code:

*Controls :* This directory consists of the utils folder, which consists of a very rudimentary API of the setpoint navigation functionality of the PX4 Flight Stack for the Gazebo SITL Simulation. It has an "autopilot" class consisting of various useful functions.

*Path Planning :* This directory contains the Autopilot directory, that consists of the Autopilot Python code, which is a class containing the various functions used for navigating through the frames. It depends on the Controls Package to provide the nessecary Setpoint Navigation functions to carry out its mission.
    
  *Perception :* This directory consists of the Frame detection package, whose scripts are written using the OpenCV library in Python.

*Localization :* This directory consists of the ORBSLAM2 package, that we directly used for our purposes, after tweaking out the parameters to suit our purpose. It works by detecting multiple features in the environment and then mapping them and itself with respect to them, thus providing very high accuracy indoors position data. 

