### 41014 Sensors and Control for Mechatronic Systems - Spring 2022 - Project

# Description
The aim of our project is to set up a simulated environment where we place two turtlebot’s. One of the turtlebot’s will be designated as the leader and this will be controlled through the use of the ‘IJKL” keys from the terminal. The second turtlebot’s will be designated as the follower, this will follow based on the information passed back from the onboard sensors and the control algorithm passed to the turtlebot through matlab.

## Requirements
# System 
- Ubuntu 18.04  
- ROS Melodic or Noetic 
- MATLAB 2022a
# dependencies
-ROBOTIS-GIT Turtlebot3 msgs - https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git  
-ROBOTIS-GIT Turtlebot3 - https://github.com/ROBOTIS-GIT/turtlebot3
-ROBOTIS-GIT Turtlebot3 simulations - https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git 

## Running the project 
#Installing turtlebot simulations 
1. Clone and build repositories into catkin workspace 
- open a new terminal 
cd ~/catkin_ws/src/  
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git  
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git  
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git  
cd ~/catkin_ws   
catkin_make   
source devel/setup.bash

#Installing follower environment
1. Clone the mulit_robot repository to the /src directory of your catkin workspace

#Running the project 
1. Ensure you have the correct repositories as listed above 
2. Open new terminal
3. run command "roslaunch multi_robot main.launch
4. Open matlab and run xxxxxx 
5. Open a new terminal 
6. Run command "rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/leader/cmd_vel"
7. Adjust the velocity of the leader robot;
- press x until linear "speed" is ~ 0.08
- press c until the angular speed “turn” is ~ 0.5
8. Use "ijkl" to control the leader robot 

#Collaborators
- Reamy Muong
- Yu Yang
- Douglas Bacon  


