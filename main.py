import numpy as np
from plot_utils import plot_trajectories_and_theta
from ekf_control_module import manrobot_with_control

# ekf_control_module.py
# Author: Akash Deshapathi
# Description: The main module for the EKF control simulation

"""
Runs the main simulation loop.
Generates :
    A) A plot showing 𝑥 vs. 𝑦 and 𝑥ℎ vs. 𝑦ℎ - without the EKF, superimposed on 
        each other using different colors/markers/line styles. Also shows the start, and 
        the desired end point clearly on this plot. (10 Points)
    B) Above plots – with an EKF.
    C) Plots (separately) of 𝜃 vs. 𝑡 with and without the EKF.
"""


def main():
    # Parameters
    mu = [0.05, 0.05, 0.05, 0.05]
    eta = [0.1, 0.1, 0.1, 0.1]
    t0 = 0  # Start time
    ts = 0.05  # Time step
    tend = 1000  # End time
    x0 = [1, 1, 0]  # Initial state [x, y, theta]
    h = 0.2  # offset
    xr = 5  # Desired x-coordinate
    yr = 0.6  # Desired y-coordinate
    kend = 1000  # Number of simulation steps
    F = 100 + np.zeros(kend)  # Throttle command as per your MATLAB code
    m = 200  # Mass of the robot

    # Run the simulations and plot the results
    for i in range(4):  # For four separate simulation runs
        trajectory_control, trajectory_ekf = manrobot_with_control(
            t0, ts, tend, x0, h, xr, yr, mu, eta, ts, F, m, kend)

        # Plot the results for each run
        plot_trajectories_and_theta(trajectory_control, trajectory_ekf, x0, xr, yr, t0, tend, ts)


# Call the main function
if __name__ == "__main__":
    main()
