# Robot Navigation with EKF Control
Overview
This project implements a control system for a robotic navigation task using Extended Kalman Filter (EKF).
The goal is to navigate a robot from an initial position to a desired target position in a two-dimensional space, while accounting for process and measurement noise.

Features
Simulation of robot movement using feedback linearization control.
Implementation of Extended Kalman Filter for state estimation under noise.
Visualization of the robot's trajectory and orientation over time.
Requirements
Python 3.x
NumPy
Matplotlib
Installation
Clone the repository to your local machine: git clone https://your-repository-link.git

Usage: 
Navigate to the project directory and run the main Python script:

python main.py
This will execute the simulation and display the resulting plots.

Files in the Repository
main.py: The main script that runs the simulation and visualization.
ekf_control_module.py: Module containing the manrobot_with_control function for robot control and EKF implementation.
plot_utils.py: Utility script for generating plots of the robot's trajectory and orientation.
Configuration
You can adjust the simulation parameters in main.py, such as the initial state, target position, time step, and total simulation time.

Author
Akash Deshapathi
