import matplotlib.pyplot as plt
import os
import sys
import numpy as np

def jacobian(theta1, theta2, theta3, theta4):
    J = np.array([
        [-0.4*(np.sin(theta1)*np.cos(theta2)*np.cos(theta3) + np.sin(theta3)*np.cos(theta1))*np.cos(theta4) + 0.4*np.sin(theta1)*np.sin(theta2)*np.sin(theta4) - 0.55*np.sin(theta1)*np.sin(theta2) - 0.045*np.sin(theta1)*np.cos(theta2)*np.cos(theta3) - 0.045*np.sin(theta3)*np.cos(theta1), 
         (-0.4*np.sin(theta2)*np.cos(theta3)*np.cos(theta4) - 0.045*np.sin(theta2)*np.cos(theta3) - 0.4*np.sin(theta4)*np.cos(theta2) + 0.55*np.cos(theta2))*np.cos(theta1), 
         -0.4*np.sin(theta1)*np.cos(theta3)*np.cos(theta4) - 0.045*np.sin(theta1)*np.cos(theta3) - 0.4*np.sin(theta3)*np.cos(theta1)*np.cos(theta2)*np.cos(theta4) - 0.045*np.sin(theta3)*np.cos(theta1)*np.cos(theta2), 
         0.4*np.sin(theta1)*np.sin(theta3)*np.sin(theta4) - 0.4*np.sin(theta2)*np.cos(theta1)*np.cos(theta4) - 0.4*np.sin(theta4)*np.cos(theta1)*np.cos(theta2)*np.cos(theta3)],
        
        [-0.4*(np.sin(theta1)*np.sin(theta3) - np.cos(theta1)*np.cos(theta2)*np.cos(theta3))*np.cos(theta4) - 0.045*np.sin(theta1)*np.sin(theta3) - 0.4*np.sin(theta2)*np.sin(theta4)*np.cos(theta1) + 0.55*np.sin(theta2)*np.cos(theta1) + 0.045*np.cos(theta1)*np.cos(theta2)*np.cos(theta3), 
         (-0.4*np.sin(theta2)*np.cos(theta3)*np.cos(theta4) - 0.045*np.sin(theta2)*np.cos(theta3) - 0.4*np.sin(theta4)*np.cos(theta2) + 0.55*np.cos(theta2))*np.sin(theta1), 
         -0.4*np.sin(theta1)*np.sin(theta3)*np.cos(theta2)*np.cos(theta4) - 0.045*np.sin(theta1)*np.sin(theta3)*np.cos(theta2) + 0.4*np.cos(theta1)*np.cos(theta3)*np.cos(theta4) + 0.045*np.cos(theta1)*np.cos(theta3), 
         -0.4*np.sin(theta1)*np.sin(theta2)*np.cos(theta4) - 0.4*np.sin(theta1)*np.sin(theta4)*np.cos(theta2)*np.cos(theta3) - 0.4*np.sin(theta3)*np.sin(theta4)*np.cos(theta1)],

        [0, 
         0.4*np.sin(theta2)*np.sin(theta4) - 0.55*np.sin(theta2) - 0.4*np.cos(theta2)*np.cos(theta3)*np.cos(theta4) - 0.045*np.cos(theta2)*np.cos(theta3), 
         (0.4*np.cos(theta4) + 0.045)*np.sin(theta2)*np.sin(theta3), 
         0.4*np.sin(theta2)*np.sin(theta4)*np.cos(theta3) - 0.4*np.cos(theta2)*np.cos(theta4)],

        [0, 
         -np.sin(theta1), 
         np.sin(theta2)*np.cos(theta1), 
         -np.sin(theta1)*np.cos(theta3) - np.sin(theta3)*np.cos(theta1)*np.cos(theta2)],

        [0, 
         np.cos(theta1), 
         np.sin(theta1)*np.sin(theta2), 
         -np.sin(theta1)*np.sin(theta3)*np.cos(theta2) + np.cos(theta1)*np.cos(theta3)],

        [1, 
         0, 
         np.cos(theta2), 
         np.sin(theta2)*np.sin(theta3)]
    ])
    
    return J

def calculate_cartesian_force(dynamics_data, joint_angles):
    theta1, theta2, theta3, theta4 = joint_angles
    # Get the Jacobian matrix based on the current joint angles
    J = jacobian(theta1, theta2, theta3, theta4)
    
    # External torque from dynamics data (assuming external torque is stored in dynamics_data['applied external torque'])
    external_torque = dynamics_data['applied external torque']

    # Compute the pseudo-inverse of the Jacobian matrix
    J_pinv = np.linalg.pinv(J)

    # Transpose of the pseudo-inverse of the Jacobian matrix
    J_pinv_T = J_pinv.T

    # Calculate Cartesian force by multiplying J_pinv_T with external torque
    cartesian_force = np.dot(J_pinv_T, external_torque.T[:4])  # Ensure torque is correctly transposed

    return cartesian_force.T

def read_config(file_path):
    kinematics_vars = []
    dynamics_vars = []

    try:
        with open(file_path, 'r') as file:
            for line in file:
                line = line.strip()
                if line.startswith("Kinematics data:"):
                    kinematics_vars = line.split(":")[1].strip().split(", ")
                elif line.startswith("Dynamics data:"):
                    dynamics_vars = line.split(":")[1].strip().split(", ")
    except FileNotFoundError:
        print(f"Error: The file {file_path} was not found.")
    except Exception as e:
        print(f"An error occurred while reading {file_path}: {e}")

    return kinematics_vars, dynamics_vars

def read_data(file_path, variable_names):
    data_dict = {name: [] for name in variable_names}

    try:
        with open(file_path, 'r') as file:
            for line in file:
                values = line.strip().split(",")  
                
                if len(values) >= 1:  # At least one value for time
                    data_dict[variable_names[0]].append(float(values[0]))  # Time
                
                for i in range(1, len(variable_names)):
                    idx = (i - 1) * 3 + 1  # Adjust index for the rest of the variables
                    if idx < len(values):
                        data_dict[variable_names[i]].append([
                            float(values[idx]),
                            float(values[idx + 1]) if idx + 1 < len(values) else None,
                            float(values[idx + 2]) if idx + 2 < len(values) else None
                        ])

    except FileNotFoundError:
        print(f"Error: The file {file_path} was not found.")
    except ValueError as ve:
        print(f"ValueError: {ve} in file {file_path} with line '{line}'")
    except Exception as e:
        print(f"An error occurred while reading {file_path}: {e}")

    # Convert lists to NumPy arrays for easier manipulation
    for key in data_dict:
        data_dict[key] = np.array(data_dict[key])
        
    return data_dict

def plot_data(kinematics_data, dynamics_data, num_data):

    plt.figure(figsize=(12, 12))

    kinematics_time = kinematics_data['time']
    # Subplot for Joint 2 Position
    plt.subplot(num_data, 2, 1)
    plt.plot(kinematics_time, kinematics_data['desired joint pos'][:, 1], label='Desired Joint 2 Position', linestyle='--')
    plt.plot(kinematics_time, kinematics_data['feedback joint pos'][:, 1], label='Feedback Joint 2 Position', linestyle='-')
    plt.title('Joint 2 Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend()

    # Subplot for Joint 4 Position
    plt.subplot(num_data, 2, 2)
    plt.plot(kinematics_time, kinematics_data['desired joint pos'][:, 3], label='Desired Joint 4 Position', linestyle='--')
    plt.plot(kinematics_time, kinematics_data['feedback joint pos'][:, 3], label='Feedback Joint 4 Position', linestyle='-')
    plt.title('Joint 4 Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend()

    # Subplot for Joint 2 Velocity
    plt.subplot(num_data, 2, 3)
    plt.plot(kinematics_time, kinematics_data['desired joint vel'][:, 1], label='Desired Joint 2 Velocity', linestyle='--')
    plt.plot(kinematics_time, kinematics_data['feedback joint vel'][:, 1], label='Feedback Joint 2 Velocity', linestyle='-')
    plt.title('Joint 2 Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.legend()

    # Subplot for Joint 4 Velocity
    plt.subplot(num_data, 2, 4)
    plt.plot(kinematics_time, kinematics_data['desired joint vel'][:, 3], label='Desired Joint 4 Velocity', linestyle='--')
    plt.plot(kinematics_time, kinematics_data['feedback joint vel'][:, 3], label='Feedback Joint 4 Velocity', linestyle='-')
    plt.title('Joint 4 Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.legend()

    # Subplot for Joint 2 Acceleration
    plt.subplot(num_data, 2, 5)
    plt.plot(kinematics_time, kinematics_data['desired joint acc'][:, 1], label='Desired Joint 2 Acceleration', linestyle='--')
    plt.plot(kinematics_time, kinematics_data['feedback joint acc'][:, 1], label='Feedback Joint 2 Acceleration', linestyle='-')
    plt.title('Joint 2 Acceleration')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration')
    plt.legend()

    # Subplot for Joint 4 Acceleration
    plt.subplot(num_data, 2, 6)
    plt.plot(kinematics_time, kinematics_data['desired joint acc'][:, 3], label='Desired Joint 4 Acceleration', linestyle='--')
    plt.plot(kinematics_time, kinematics_data['feedback joint acc'][:, 3], label='Feedback Joint 4 Acceleration', linestyle='-')
    plt.title('Joint 4 Acceleration')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration')
    plt.legend()

    dynamics_time = dynamics_data['time']

    plt.subplot(num_data, 2, 7)
    plt.plot(dynamics_time, dynamics_data['wam joint torque input'][:, 1])
    plt.title('Joint 2 WAM Torque Input')
    plt.xlabel('Time (s)')
    plt.ylabel('Torque Input')

    plt.subplot(num_data, 2, 8)
    plt.plot(dynamics_time, dynamics_data['wam joint torque input'][:, 3])
    plt.title('Joint 4 WAM Torque Input')
    plt.xlabel('Time (s)')
    plt.ylabel('Torque Input')

    plt.subplot(num_data, 2, 9)
    plt.plot(dynamics_time, dynamics_data['wam gravity input'][:, 1])
    plt.title('Joint 2 WAM Gravity Input')
    plt.xlabel('Time (s)')
    plt.ylabel('Gravity Input')

    plt.subplot(num_data, 2, 10)
    plt.plot(dynamics_time, dynamics_data['wam gravity input'][:, 3])
    plt.title('Joint 4 WAM Gravity Input')
    plt.xlabel('Time (s)')
    plt.ylabel('Gravity Input')

    plt.subplot(num_data, 2, 11)
    plt.plot(dynamics_time, dynamics_data['dynamic feed forward'][:, 1])
    plt.title('Joint 2 WAM Dynamic Feed Forward')
    plt.xlabel('Time (s)')
    plt.ylabel('Dynamic Feed Forward Input')

    plt.subplot(num_data, 2, 12)
    plt.plot(dynamics_time, dynamics_data['dynamic feed forward'][:, 3])
    plt.title('Joint 4 WAM Dynamic Feed Forward')
    plt.xlabel('Time (s)')
    plt.ylabel('Dynamic Feed Forward Input')

    plt.subplot(num_data, 2, 13)
    plt.plot(dynamics_time, dynamics_data['inverse dynamic'][:, 1])
    plt.title('Joint 2 WAM Inverse Dynamic')
    plt.xlabel('Time (s)')
    plt.ylabel('Inverse Dynamic')

    plt.subplot(num_data, 2, 14)
    plt.plot(dynamics_time, dynamics_data['inverse dynamic'][:, 3])
    plt.title('Joint 4 WAM Inverse Dynamic')
    plt.xlabel('Time (s)')
    plt.ylabel('Inverse Dynamic')

    plt.subplot(num_data, 2, 15)
    plt.plot(dynamics_time, dynamics_data['applied external torque'][:, 1], label='Applied Joint 2 External Torque', linestyle='--')
    plt.plot(dynamics_time, dynamics_data['calculated external torque'][:, 1], label='Calculated Joint 2 External Torque', linestyle='-')
    plt.title('Joint 2 External Torque')
    plt.xlabel('Time (s)')
    plt.ylabel('External Torque')
    plt.legend()

    plt.subplot(num_data, 2, 16)
    plt.plot(dynamics_time, dynamics_data['applied external torque'][:, 3], label='Applied Joint 4 External Torque', linestyle='--')
    plt.plot(dynamics_time, dynamics_data['calculated external torque'][:, 3], label='Calculated Joint 4 External Torque', linestyle='-')
    plt.title('Joint 4 External Torque')
    plt.xlabel('Time (s)')
    plt.ylabel('External Torque')
    plt.legend()

    plt.show()

def calculate_nrmse(desired, feedback, normalization='range'):
    """
    Calculate the Normalized Root Mean Square Error (nRMSE).

    Parameters:
    desired (array-like): Desired values.
    feedback (array-like): Feedback values.
    normalization (str): 'range' to normalize by the range of desired values,
                         'mean' to normalize by the mean of desired values.

    Returns:
    float: Normalized Root Mean Square Error.
    """
    # Calculate RMSE
    rmse = np.sqrt(np.mean((desired - feedback) ** 2))
    
    # Normalize RMSE
    if normalization == 'range':
        range_desired = np.max(desired) - np.min(desired)
        nrmse = rmse / range_desired if range_desired != 0 else float('inf')
    elif normalization == 'mean':
        mean_desired = np.mean(desired)
        nrmse = rmse / mean_desired if mean_desired != 0 else float('inf')
    else:
        raise ValueError("Normalization method must be 'range' or 'mean'.")
    
    return nrmse

def calculate_errors(kinematics_data, dynamics_data):
    
    pos_des_2 = kinematics_data['desired joint pos'][:, 1]
    pos_feedback_2 = kinematics_data['feedback joint pos'][:, 1]
    pos_nrmse_2 = calculate_nrmse(pos_des_2, pos_feedback_2)

    pos_des_4 = kinematics_data['desired joint pos'][:, 3]
    pos_feedback_4 = kinematics_data['feedback joint pos'][:, 3]
    pos_nrmse_4 = calculate_nrmse(pos_des_4, pos_feedback_4)

    applied_extorq_2 = dynamics_data['applied external torque'][:, 1]
    calculated_extorq_2 = dynamics_data['calculated external torque'][:, 1]
    extorq_measurement_nrmse_2 = calculate_nrmse(applied_extorq_2, calculated_extorq_2)

    applied_extorq_4 = dynamics_data['applied external torque'][:, 3]
    calculated_extorq_4 = dynamics_data['calculated external torque'][:, 3]
    extorq_measurement_nrmse_4 = calculate_nrmse(applied_extorq_4, calculated_extorq_4)

    print("pos_nrmse_2: ", pos_nrmse_2)
    print("pos_nrmse_4: ", pos_nrmse_4)

    print("extorq_measurement_nrmse_2: ", extorq_measurement_nrmse_2)
    print("extorq_measurement_nrmse_4: ", extorq_measurement_nrmse_4)


def main(folder_name):
    base_folder = './.data'  # Adjust this to your actual folder structure
    folder_path = os.path.join(base_folder, folder_name)

    config_file = os.path.join(folder_path, 'config.txt')
    kinematics_file = os.path.join(folder_path, 'kinematics.txt')
    dynamics_file = os.path.join(folder_path, 'dynamics.txt')

    # Read variable names from config
    kinematics_vars, dynamics_vars = read_config(config_file)

    # Read data from kinematics and dynamics files
    kinematics_data = read_data(kinematics_file, kinematics_vars)
    dynamics_data = read_data(dynamics_file, dynamics_vars)

    joint_angles = kinematics_data['feedback joint pos'][-1, :4]  # Taking the last feedback joint position

    cal_extorq = dynamics_data[dynamics_vars[2]] - dynamics_data[dynamics_vars[3]] - dynamics_data[dynamics_vars[5]]
    dynamics_data['calculated external torque'] = cal_extorq
    dynamics_vars.append("calculated external torque")
    num_data = 3 + len(dynamics_vars) - 1
    
    plot_data(kinematics_data, dynamics_data, num_data)
    calculate_errors(kinematics_data, dynamics_data)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <folder_name>")
    else:
        main(sys.argv[1])
