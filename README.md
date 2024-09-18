# dam_cleaning

## Description
This project focuses on the path planning and control system for a dam cleaning robot using ROS (Robot Operating System). The repository includes various coverage planning strategies for cleaning the dam area, along with world models and launch files for simulation. The project utilizes TurtleBot3 and custom world models designed for simulation purposes.

## Repository Structure
- launch/: Contains launch files for various robot and simulation configurations.
- strategy/: Includes different path planning and control strategies for the dam cleaning robot, including Python scripts and launch files.
- world/: Contains world models for the dam simulation and related launch files.

## Environment Setup
### 1. Create a Workspace
1. Create a new workspace
```
mkdir -p ~/dam_cleaning_ws/src
cd ~/dam_cleaning_ws/src
```
2. Clone this repository into the workspace:
```
git clone https://github.com/FaaiqMastyaga/dam_cleaning.git
```
3. Return to the workspace directory and build the workspace:
```
cd ~/dam_cleaning_ws
catkin_make
```

### 2. Install Packages
Ensure you have ROS Noetic and the required dependencies installed. If not, follow the ROS installation guide.
Dont' forget to change directory to the workspace:
```
cd ~/dam_cleaning_ws
```
Install the necessary ROS packages for this project:
```
sudo apt-get install ros-noetic-turtlebot3
sudo apt-get install ros-noetic-turtlebot3-simulations
```

### 3. Set Environment Variables  
Set the TurtleBot3 model environment variable:
```
export TURTLEBOT3_MODEL=burger
```
You can add this line to your ~/.bashrc or ~/.zshrc file to set it automatically for each new terminal session.

## Running the Program
### 1. Launch Coverage Planner
To start the coverage planner node:
```
roslaunch damc_strategy coverage_planner.launch
```
### 2. Launch Dam World Simulation
To start the dam world simulation:
```
roslaunch damc_world turtlebot3_dam_world.launch
```
### 3. Launch SLAM and Move Base
To run the SLAM and Move Base nodes:
```
roslaunch turtlebot3 turtlebot3_slam.launch
roslaunch turtlebot3 move_base.launch
```
### 4. Running All Components Simultaneously
To run all the components (Coverage Planner, Dam World Simulation, SLAM, and Move Base) together, you can use:
```
roslaunch damc_launch turtlebot3_auto_dam.launch
```