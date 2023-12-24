import numpy as np
from scipy.optimize import minimize
import pandas as pd
import os

script_directory = os.path.dirname(os.path.abspath(__file__))

# Load data
data = pd.read_excel(script_directory+"/data/preprocess_sliding_mode_controller_data.xlsx", sheet_name="preprocess_data")

# Extract data
timestamp = data["timestamp"].values
trolley_motor_voltage = data["trolley_motor_voltage"].values
hoist_motor_voltage = data["hoist_motor_voltage"].values
trolley_position_triple_dot = data["trolley_position_third_derivative"].values
cable_length_triple_dot = data["cable_length_third_derivative"].values
trolley_position_double_dot = data["trolley_position_second_derivative"].values
cable_length_double_dot = data["cable_length_second_derivative"].values
trolley_position_dot = data["trolley_position_first_derivative"].values
cable_length_dot = data["cable_length_first_derivative"].values
trolley_position = data["trolley_position"].values
cable_length = data["cable_length"].values
sway_angle_double_dot = data["sway_angle_second_derivative"].values
sway_angle_dot = data["sway_angle_first_derivative"].values
sway_angle = data["sway_angle"].values

delta_t = timestamp[1] - timestamp[0]

mc = 1.128
mt = 2.0
g = 9.81

# Motor 1
L1 = 0.0001048
R1 = 0.39
rp1 = 0.015

# Motor 2
L2 = 0.0073
R2 = 9.5
rp2 = 0.015


def gantry_model(
    parameters,
    trolley_motor_voltage,
    hoist_motor_voltage,
    delta_t,
):
    # Unpack parameters
    bt, br, b1, b2, J1, J2, Kt1, Kt2, Ke1, Ke2 = parameters
    print("parameters: ", parameters)

    # Initialize variables
    x = trolley_position[0]
    l = cable_length[0]
    q = np.array([[x], [l]])
    q_dot = np.array([[0.0], [0.0]])
    q_double_dot = np.array([[0.0], [0.0]])

    l_dot = 0.0
    theta = 0.0    
    theta_dot = 0.0
    theta_double_dot = 0.0

    matrix_A = np.matrix([[0.0, 0.0], [0.0, 0.0]])
    matrix_B = np.matrix([[0.0, 0.0], [0.0, 0.0]])
    matrix_C = np.matrix([[0.0, 0.0], [0.0, 0.0]])
    matrix_D = np.matrix([[0.0], [0.0]])
    matrix_E = np.matrix([[0.0], [0.0]])
    matrix_F = np.matrix([[0.0], [0.0]])

    for i in range(len(trolley_motor_voltage)):
        # print("i: ", i)
        # Update motor voltage
        motor_voltage = np.array([trolley_motor_voltage, hoist_motor_voltage])
        
        # Update matrix A
        matrix_A[0, 0] = L1 * rp1 * (mt + mc * np.sin(theta) ** 2) / Kt1 + J1 * L1 / (
            Kt1 * rp1
        )
        matrix_A[0, 1] = -L1 * mc * rp1 * np.sin(theta) / Kt1
        matrix_A[1, 0] = -L2 * mc * rp2 * np.sin(theta) / Kt2
        matrix_A[1, 1] = L2 * mc * rp2 / Kt2 + J2 * L2 / (Kt2 * rp2)

        # Update matrix B
        matrix_B[0, 0] = (
            +L1 * mc * rp1 * np.sin(2 * theta) * theta_dot / Kt1
            + R1 * rp1 * (mt + mc * np.sin(theta) ** 2) / Kt1
            + L1 * bt * rp1 / Kt1
            + J1 * R1 / (Kt1 * rp1)
            + L1 * b1 / (Kt1 * rp1)
        )
        matrix_B[0, 1] = (
            -L1 * mc * rp1 * np.cos(theta) * theta_dot / Kt1
            - R1 * mc * rp1 * np.sin(theta) / Kt1
        )
        matrix_B[1, 0] = (
            -L2 * mc * rp2 * np.cos(theta) * theta_dot / Kt2
            - R2 * mc * rp2 * np.sin(theta) / Kt2
        )
        matrix_B[1, 1] = (
            +L2 * br * rp2 / Kt2
            + R2 * mc * rp2 / Kt2
            + J2 * R2 / (Kt2 * rp2)
            + L2 * b2 / (Kt2 * rp2)
        )

        # Update matrix C
        matrix_C[0, 0] = R1 * bt * rp1 / Kt1 + Ke1 / rp1 + R1 * b1 / (Kt1 * rp1)
        matrix_C[1, 1] = R2 * br * rp2 / Kt2 + Ke2 / rp2 + R2 * b2 / (Kt2 * rp2)

        # Update matrix D
        matrix_D[0, 0] = 2 * L1 * mc * rp1 * l * np.sin(theta) * theta_dot / Kt1
        matrix_D[1, 0] = -2 * L2 * mc * rp2 * l * theta_dot / Kt2

        # Update matrix E
        matrix_E[0, 0] = (
            L1 * mc * rp1 * l * np.cos(theta) * theta_dot**2 / Kt1
            + L1 * g * mc * rp1 * np.cos(2 * theta) / Kt1
            + L1 * mc * rp1 * np.sin(theta) * l_dot * theta_dot / Kt1
            + R1 * mc * rp1 * l * np.sin(theta) * theta_dot / Kt1
        )
        matrix_E[1, 0] = (
            +L2 * g * mc * rp2 * np.sin(theta) / Kt2
            - L2 * mc * rp2 * l_dot * theta_dot / Kt2
            - R2 * mc * rp2 * l * theta_dot / Kt2
        )

        # Update matrix F
        matrix_F[0, 0] = R1 * g * mc * rp1 * np.sin(theta) * np.cos(theta) / Kt1
        matrix_F[1, 0] = -R2 * g * mc * rp2 * np.cos(theta) / Kt2

        q_triple_dot = np.matmul(
                np.linalg.inv(matrix_A),
                (
                    motor_voltage
                    - (
                        matrix_B * q_double_dot
                        + matrix_C * q_dot
                        + matrix_D * theta_double_dot
                        + matrix_E * theta_dot
                        + matrix_F
                    )
                ),
            )
        
        q_double_dot = q_double_dot + q_triple_dot * delta_t
        q_dot = q_dot + q_double_dot * delta_t
        q = q + q_dot * delta_t

        theta_double_dot = (
            np.cos(theta) * q_double_dot[0,0]
            - 2 * l_dot * theta_dot
            - np.sin(theta) * g
        ) / l

        theta_dot = theta_dot + theta_double_dot * delta_t
        theta = theta + theta_dot * delta_t

        l = q[1,0]
        l_dot = q_dot[1,0]

    print("q: ", q)
    return q

def objective_function(parameters, *args):
    trolley_motor_voltage, hoist_motor_voltage, delta_t = args
    q = gantry_model(parameters, trolley_motor_voltage, hoist_motor_voltage, delta_t)
    
    error = np.concatenate((trolley_position - q[0,0], cable_length - q[1,0]))

    print("error: ", error)
    return np.sum(error**2)

# Initial guess
bt = 0.5
br = 10.0
b1 = 0.0004
b2 = 0.0008
J1 = 0.00008859375
J2 = 0.000015625
Kt1 = 20.03
Kt2 = 33.54
Ke1 = 0.00818
Ke2 = 0.00818
parameters = [bt, br, b1, b2, J1, J2, Kt1, Kt2, Ke1, Ke2]

# Set bounds
bounds = (
    (0.0, 10),  # bt
    (0.0, 100),  # br
    (0.0, 0.1),  # b1
    (0.0, 0.1),  # b2
    (0.0, 0.0001),  # J1
    (0.0, 0.0001),  # J2
    (0.0, 100),  # Kt1
    (0.0, 100),  # Kt2
    (0.0, 0.1),  # Ke1
    (0.0, 0.1),  # Ke2
)

# Optimize
result = minimize(
    objective_function,
    parameters,
    args=(trolley_motor_voltage, hoist_motor_voltage, delta_t),
    method="SLSQP",
    bounds=bounds,
)

# Print results
print("Optimization results:")
print("bt: ", result.x[0])
print("br: ", result.x[1])
print("b1: ", result.x[2])
print("b2: ", result.x[3])
print("J1: ", result.x[4])
print("J2: ", result.x[5])
print("Kt1: ", result.x[6])
print("Kt2: ", result.x[7])
print("Ke1: ", result.x[8])
print("Ke2: ", result.x[9])
print("Optimization status: ", result.success)
print("Optimal cost: ", result.fun)
print("Number of iterations performed: ", result.nit)

