import numpy as np

# ekf_control_module.py
# Author: Akash Deshapathi
# Description: Module containing the EKF-based control algorithm for robot navigation.

"""
Generates the function comment for the given function body.

Parameters:
- t0 (float): The initial time.
- ts (float): The time step.
- tend (float): The end time.
- x0 (ndarray): The initial state.
- h (float): The distance between the robot's center of mass and the wheel axis.
- xr (float): The x-coordinate of the target position.
- yr (float): The y-coordinate of the target position.
- mu (ndarray): The noise parameters for the process model.
- eta (ndarray): The noise parameters for the measurement model.
- T (float): The sampling time of the process model.
- F (ndarray): The control input applied to the robot.
- m (float): The mass of the robot.
- kend (int): The number of time steps.

Returns:
- xtemp_control (ndarray): The control-based states.
- xtemp_ekf (ndarray): The EKF-based states.
"""


def manrobot_with_control(t0, ts, tend, x0, h, xr, yr, mu, eta, T, F, m, kend):
    # Time vector
    tend = kend
    time = np.arange(t0, tend + ts, ts)
    num_steps = len(time)

    # Initialize the state matrices for control and EKF estimates
    xtemp_control = np.zeros((num_steps, 6))  # Control-based states
    xtemp_ekf = np.zeros((num_steps, 6))  # EKF-based states
    xtemp_control[0, :3] = x0
    xtemp_ekf[0, :3] = x0

    # Initialize EKF variables
    x_ekf, y_ekf, v_ekf, th_ekf = np.zeros(num_steps), np.zeros(num_steps), np.zeros(num_steps), np.zeros(num_steps)
    Pkplus = np.diag([10, 10, 10, 10])

    # Define a threshold for stopping near the target
    stopping_threshold = 0.1  # Adjust this value based on desired proximity

    # Control and EKF loop
    for k in range(1, num_steps):
        # Current state from control
        x_control, y_control, theta_control = xtemp_control[k - 1, :3]
        v_control = v_ekf[k - 1] if k > 1 else 0

        # Calculate distance to target
        distance_to_target = np.sqrt((x_control - xr) ** 2 + (y_control - yr) ** 2)

        # Stopping criterion
        if distance_to_target < stopping_threshold:
            v_control = 0  # Stop the robot

        # Update velocity using the dynamics model
        v_control += ts * (F[k - 1] / m) if v_control > 0 else 0

        # Calculate control input u using feedback linearization
        e_control = np.array([x_control - xr, y_control - yr])
        R_control = np.array([[np.cos(theta_control), -np.sin(theta_control)],
                              [np.sin(theta_control), h * np.cos(theta_control)]])
        K_control = np.array([[0.5, 0], [0, 0.5]])
        g_control = -np.linalg.inv(R_control) @ K_control @ e_control
        w_control = g_control[1]

        # Update the control-based state using the updated velocity
        xtemp_control[k, 0] = x_control + ts * v_control * np.cos(theta_control)
        xtemp_control[k, 1] = y_control + ts * v_control * np.sin(theta_control)
        xtemp_control[k, 2] = theta_control + ts * w_control
        xtemp_control[k, 4] = xtemp_control[k, 0] + h * np.cos(xtemp_control[k, 2])  # xh
        xtemp_control[k, 5] = xtemp_control[k, 1] + h * np.sin(xtemp_control[k, 2])  # yh

        # EKF Estimation
        # Prediction
        Fk = np.array([[1, 0, T * np.cos(th_ekf[k - 1]), -T * v_ekf[k - 1] * np.sin(th_ekf[k - 1])],
                       [0, 1, T * np.sin(th_ekf[k - 1]), T * v_ekf[k - 1] * np.cos(th_ekf[k - 1])],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
        Lk, Hk = np.eye(4), np.eye(4)
        Qk = 2 * np.diag(mu)
        Rk = 2 * np.diag(eta)
        Mk = np.eye(4)

        Pkmin = Fk @ Pkplus @ Fk.T + Lk @ Qk @ Lk.T
        Kk = Pkmin @ Hk.T @ np.linalg.inv(Hk @ Pkmin @ Hk.T + Mk @ Rk @ Mk.T)

        # Measurement update using the noise as eta
        zk = np.array([xtemp_control[k - 1, 0] + 2 * eta[0] * np.random.randn(),
                       xtemp_control[k - 1, 1] + 2 * eta[1] * np.random.randn(),
                       v_control + 2 * eta[2] * np.random.randn(),
                       w_control + 2 * eta[3] * np.random.randn()])
        xhkmin = np.array([x_ekf[k - 1], y_ekf[k - 1], v_ekf[k - 1], th_ekf[k - 1]])
        xhkpl = xhkmin + Kk @ (zk - xhkmin)

        # Update the EKF-based state
        x_ekf[k], y_ekf[k], v_ekf[k], th_ekf[k] = xhkpl
        xtemp_ekf[k, 0] = x_ekf[k]
        xtemp_ekf[k, 1] = y_ekf[k]
        xtemp_ekf[k, 2] = th_ekf[k]
        xtemp_ekf[k, 4] = xtemp_ekf[k, 0] + h * np.cos(xtemp_ekf[k, 2])  # xh
        xtemp_ekf[k, 5] = xtemp_ekf[k, 1] + h * np.sin(xtemp_ekf[k, 2])  # yh

        Pkplus = np.linalg.inv(np.linalg.inv(Pkmin) + Hk.T @ np.linalg.inv(Rk) @ Hk)

    return xtemp_control, xtemp_ekf
