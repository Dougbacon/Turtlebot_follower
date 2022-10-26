### 41014 Sensors and Control for Mechatronic Systems - Spring 2022 - Project

# Description
The aim of our project is to set up a simulated environment where we place two turtlebot’s. One of the turtlebot’s will be designated as the leader and this will be controlled through the use of the ‘IJKL” keys from the terminal. The second turtlebot’s will be designated as the follower, this will follow based on the information passed back from the onboard sensors and the control algorithm passed to the turtlebot through matlab.

## Requirements
# System 
1. Ubuntu 18.04
2. ROS Melodic or Noetic 
3. MATLAB 2022a
- ROS Toolbox
- Robotics System Toolbox
- Image Processing Toolbox
- Computer Vision Toolbox 
# Dependencies
- ROBOTIS-GIT Turtlebot3 msgs - https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
- ROBOTIS-GIT Turtlebot3 - https://github.com/ROBOTIS-GIT/turtlebot3
- ROBOTIS-GIT Turtlebot3 simulations - https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git 

## Setting up the project
# Installing turtlebot simulations (if not previously installed)
1. Clone and build repositories into catkin workspace 
- open a new terminal,copying the whats within the "" into the terminal  
- "cd ~/catkin_ws/src/"
- "git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git"
- "git clone https://github.com/ROBOTIS-GIT/turtlebot3.git"
- "git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git"
- "cd ~/catkin_ws"
- "catkin_make" 
- "source devel/setup.bash"

# Installing follower environment
1. Clone the mulit_robot repository to the /src directory of your catkin workspace
- Open a new terminal 
- "cd ~/catkin_ws/src/"
- "git clone https://github.com/Dougbacon/multi_robot"

# Running the project 
1. Ensure you have the correct repositories as listed above 
2. Open new terminal
3. Run command "source ~/.bashrc"
4. Run command "roslaunch multi_robot main.launch"
5. Open matlab and add the multi_robot folder and sub folders to your path
6. Open Matlab folder within the multi_robot folder
7. Run Group7_Project2.m 
8. Open a new terminal 
9. Run command "rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/leader/cmd_vel"
10. Adjust the velocity of the leader robot;
- press x until linear "speed" is ~ 0.08
- press c until the angular speed “turn” is ~ 0.5
11. Use "ijkl" to control the leader robot 

# Collaborators
- Reamy Muong (40%)
- Yu Yang (30%)
- Douglas Bacon (30%)


