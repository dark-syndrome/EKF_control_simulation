import numpy as np
import matplotlib.pyplot as plt

# ekf_control_module.py
# Author: Akash Deshapathi
# Description: Module containing the plotting functions for ekf_control_module.py

"""
Generate plots of trajectories and theta values using the given data.

Parameters:
- trajectory_control (numpy.ndarray): The trajectory data for the control system.
- trajectory_ekf (numpy.ndarray): The trajectory data for the Extended Kalman Filter (EKF) system.
- x0 (list): The initial x and y position coordinates.
- xr (float): The x position coordinate of the end point.
- yr (float): The y position coordinate of the end point.
- t0 (float): The initial time.
- tend (float): The end time.
- ts (int): The number of time steps.

Returns:
None
"""


def plot_trajectories_and_theta(trajectory_control, trajectory_ekf, x0, xr, yr, t0, tend, ts):
    # Extracting data for plots
    x_control, y_control, theta_control = trajectory_control[:, 0], trajectory_control[:, 1], trajectory_control[:, 2]
    xh_control, yh_control = trajectory_control[:, 4], trajectory_control[:, 5]

    x_ekf, y_ekf, theta_ekf = trajectory_ekf[:, 0], trajectory_ekf[:, 1], trajectory_ekf[:, 2]
    xh_ekf, yh_ekf = trajectory_ekf[:, 4], trajectory_ekf[:, 5]

    num_steps = len(trajectory_control)*int(ts/ts)
    time = np.linspace(t0, tend, num=num_steps)

    # Plot (a) Trajectories without EKF
    plt.figure(figsize=(12, 8))
    plt.subplot(2, 2, 1)
    plt.plot(x_control, y_control, label='x vs y - Control', color='red', linestyle='-', marker='')
    plt.plot(xh_control, yh_control, label='xh vs yh - Control', color='blue', linestyle='--', marker='')
    plt.scatter(x0[0], x0[1], color='black', label='Start Point (1,1)', marker='s')
    plt.scatter(xr, yr, color='orange', label='End Point (5,0.6)', marker='*')
    plt.title('Trajectories without EKF')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.legend()
    plt.grid(True)

    # Plot (b) Trajectories with EKF
    plt.subplot(2, 2, 2)
    plt.plot(x_ekf, y_ekf, label='x vs y - EKF', color='green', linestyle='-', marker='')
    plt.plot(xh_ekf, yh_ekf, label='xh vs yh - EKF', color='purple', linestyle='--', marker='')
    plt.scatter(x0[0], x0[1], color='black', label='Start Point (1,1)', marker='s')
    plt.scatter(xr, yr, color='orange', label='End Point (5,0.6)', marker='*')
    plt.title('Trajectories with EKF')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.legend()
    plt.grid(True)

    # Plot (c) Theta vs t without EKF
    plt.subplot(2, 2, 3)
    plt.plot(time, theta_control, label='Theta - Control', color='red', linestyle='-', marker='')
    plt.title('Theta vs Time without EKF')
    plt.xlabel('Time (s)')
    plt.ylabel('Theta (radians)')
    plt.legend()
    plt.grid(True)

    # Plot (c) Theta vs t with EKF
    plt.subplot(2, 2, 4)
    plt.plot(time, theta_ekf, label='Theta - EKF', color='green', linestyle='-', marker='')
    plt.title('Theta vs Time with EKF')
    plt.xlabel('Time (s)')
    plt.ylabel('Theta (radians)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()
