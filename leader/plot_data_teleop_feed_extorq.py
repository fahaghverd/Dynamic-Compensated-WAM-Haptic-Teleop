import matplotlib.pyplot as plt
import os
import sys
import numpy as np

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
    plt.plot(kinematics_time, kinematics_data['desired joint pos'][:, 0], label='Desired Joint 2 Position', linestyle='--')
    plt.plot(kinematics_time, kinematics_data['feedback joint pos'][:, 0], label='Feedback Joint 2 Position', linestyle='-')
    plt.title('Joint 2 Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend()

    # Subplot for Joint 4 Position
    plt.subplot(num_data, 2, 2)
    plt.plot(kinematics_time, kinematics_data['desired joint pos'][:, 2], label='Desired Joint 4 Position', linestyle='--')
    plt.plot(kinematics_time, kinematics_data['feedback joint pos'][:, 2], label='Feedback Joint 4 Position', linestyle='-')
    plt.title('Joint 4 Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend()

    # Subplot for Joint 2 Velocity
    plt.subplot(num_data, 2, 3)
    plt.plot(kinematics_time, kinematics_data['desired joint vel'][:, 0], label='Desired Joint 2 Velocity', linestyle='--')
    plt.plot(kinematics_time, kinematics_data['feedback joint vel'][:, 0], label='Feedback Joint 2 Velocity', linestyle='-')
    plt.title('Joint 2 Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.legend()

    # Subplot for Joint 4 Velocity
    plt.subplot(num_data, 2, 4)
    plt.plot(kinematics_time, kinematics_data['desired joint vel'][:, 2], label='Desired Joint 4 Velocity', linestyle='--')
    plt.plot(kinematics_time, kinematics_data['feedback joint vel'][:, 2], label='Feedback Joint 4 Velocity', linestyle='-')
    plt.title('Joint 4 Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.legend()

    # Subplot for Joint 2 Acceleration
    plt.subplot(num_data, 2, 5)
    plt.plot(kinematics_time, kinematics_data['desired joint acc'][:, 0], label='Desired Joint 2 Acceleration', linestyle='--')
    plt.plot(kinematics_time, kinematics_data['feedback joint acc'][:, 0], label='Feedback Joint 2 Acceleration', linestyle='-')
    plt.title('Joint 2 Acceleration')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration')
    plt.legend()

    # Subplot for Joint 4 Acceleration
    plt.subplot(num_data, 2, 6)
    plt.plot(kinematics_time, kinematics_data['desired joint acc'][:, 2], label='Desired Joint 4 Acceleration', linestyle='--')
    plt.plot(kinematics_time, kinematics_data['feedback joint acc'][:, 2], label='Feedback Joint 4 Acceleration', linestyle='-')
    plt.title('Joint 4 Acceleration')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration')
    plt.legend()

    dynamics_time = dynamics_data['time']

    plt.subplot(num_data, 2, 7)
    plt.plot(dynamics_time, dynamics_data['wam joint torque input'][:, 0])
    plt.title('Joint 2 WAM Torque Input')
    plt.xlabel('Time (s)')
    plt.ylabel('Torque Input')

    plt.subplot(num_data, 2, 8)
    plt.plot(dynamics_time, dynamics_data['wam joint torque input'][:, 2])
    plt.title('Joint 4 WAM Torque Input')
    plt.xlabel('Time (s)')
    plt.ylabel('Torque Input')

    plt.subplot(num_data, 2, 9)
    plt.plot(dynamics_time, dynamics_data['wam gravity input'][:, 0])
    plt.title('Joint 2 WAM Gravity Input')
    plt.xlabel('Time (s)')
    plt.ylabel('Gravity Input')

    plt.subplot(num_data, 2, 10)
    plt.plot(dynamics_time, dynamics_data['wam gravity input'][:, 2])
    plt.title('Joint 4 WAM Gravity Input')
    plt.xlabel('Time (s)')
    plt.ylabel('Gravity Input')

    plt.subplot(num_data, 2, 11)
    plt.plot(dynamics_time, dynamics_data['dynamic feed forward'][:, 0])
    plt.title('Joint 2 WAM Dynamic Feed Forward')
    plt.xlabel('Time (s)')
    plt.ylabel('Dynamic Feed Forward Input')

    plt.subplot(num_data, 2, 12)
    plt.plot(dynamics_time, dynamics_data['dynamic feed forward'][:, 2])
    plt.title('Joint 4 WAM Dynamic Feed Forward')
    plt.xlabel('Time (s)')
    plt.ylabel('Dynamic Feed Forward Input')

    plt.subplot(num_data, 2, 13)
    plt.plot(dynamics_time, dynamics_data['inverse dynamic'][:, 0])
    plt.title('Joint 2 WAM Inverse Dynamic')
    plt.xlabel('Time (s)')
    plt.ylabel('Inverse Dynamic')

    plt.subplot(num_data, 2, 14)
    plt.plot(dynamics_time, dynamics_data['inverse dynamic'][:, 2])
    plt.title('Joint 4 WAM Inverse Dynamic')
    plt.xlabel('Time (s)')
    plt.ylabel('Inverse Dynamic')

    plt.subplot(num_data, 2, 15)
    plt.plot(dynamics_time, dynamics_data['applied external torque'][:, 0], label='Applied Joint 2 External Torque', linestyle='--')
    plt.plot(dynamics_time, dynamics_data['calculated external torque'][:, 0], label='Calculated Joint 2 External Torque', linestyle='-')
    plt.title('Joint 2 External Torque')
    plt.xlabel('Time (s)')
    plt.ylabel('External Torque')
    # plt.legend()

    plt.subplot(num_data, 2, 16)
    plt.plot(dynamics_time, dynamics_data['applied external torque'][:, 2], label='Applied Joint 4 External Torque', linestyle='--')
    plt.plot(dynamics_time, dynamics_data['calculated external torque'][:, 2], label='Calculated Joint 4 External Torque', linestyle='-')
    plt.title('Joint 4 External Torque')
    plt.xlabel('Time (s)')
    plt.ylabel('External Torque')
    # plt.legend()

    # Impedance Calculation and Plotting
    impedance_joint_2 = dynamics_data['impedance'][:, 0]
    impedance_joint_4 = dynamics_data['impedance'][:, 2]

    plt.subplot(num_data, 2, 17)
    plt.plot(dynamics_time, dynamics_data['PD'][:,0])
    plt.title('Joint 2 Impedance')
    plt.xlabel('Time (s)')
    plt.ylabel('Impedance')

    plt.subplot(num_data, 2, 18)
    plt.plot(dynamics_time, dynamics_data['PD'][:,2])
    plt.title('Joint 4 Impedance')
    plt.xlabel('Time (s)')
    plt.ylabel('Impedance')

    plt.subplot(num_data, 2, 17)
    plt.plot(dynamics_time, dynamics_data['PD'][:, 0])
    plt.title('Joint 2 WAM PD')
    plt.xlabel('Time (s)')
    plt.ylabel('PD')

    plt.subplot(num_data, 2, 18)
    plt.plot(dynamics_time, dynamics_data['PD'][:, 2])
    plt.title('Joint 4 WAM PD')
    plt.xlabel('Time (s)')
    plt.ylabel('PD')


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

def calculate_rms(signal):
    """
    Calculate the Root Mean Square (RMS) of a signal.

    Parameters:
    signal (array-like): Input signal values.

    Returns:
    float: RMS value of the signal.
    """
    return np.sqrt(np.nanmean(np.square(signal)))

def calculate_errors(kinematics_data, dynamics_data):
    
    pos_des_2 = kinematics_data['desired joint pos'][:, 0]
    pos_feedback_2 = kinematics_data['feedback joint pos'][:, 0]
    pos_nrmse_2 = calculate_nrmse(pos_des_2, pos_feedback_2)

    pos_des_4 = kinematics_data['desired joint pos'][:, 2]
    pos_feedback_4 = kinematics_data['feedback joint pos'][:, 2]
    pos_nrmse_4 = calculate_nrmse(pos_des_4, pos_feedback_4)

    applied_extorq_2 = dynamics_data['applied external torque'][:, 0] + dynamics_data['dynamic feed forward'][:, 0]
    calculated_extorq_2 = dynamics_data['calculated external torque'][:, 0]
    extorq_measurement_nrmse_2 = calculate_nrmse(applied_extorq_2, calculated_extorq_2)

    applied_extorq_4 = dynamics_data['applied external torque'][:, 2] + dynamics_data['dynamic feed forward'][:, 2]
    calculated_extorq_4 = dynamics_data['calculated external torque'][:, 2]
    extorq_measurement_nrmse_4 = calculate_nrmse(applied_extorq_4, calculated_extorq_4)

    impedance_joint_2 = dynamics_data['impedance'][:, 0]
    impedance_rms_2 = calculate_rms(impedance_joint_2)

    impedance_joint_4 = dynamics_data['impedance'][:, 2]
    impedance_rms_4 = calculate_rms(impedance_joint_4)

    print("pos_nrmse_2: ", pos_nrmse_2)
    print("pos_nrmse_4: ", pos_nrmse_4)

    print("extorq_measurement_nrmse_2: ", extorq_measurement_nrmse_2)
    print("extorq_measurement_nrmse_4: ", extorq_measurement_nrmse_4)

    print("impedance rms joint 2:", impedance_rms_2)
    print("impedance rms joint 4:", impedance_rms_4)



def main(folder_name):
    base_folder = './.data_ral'  # Adjust this to your actual folder structure
    folder_path = os.path.join(base_folder, folder_name)

    config_file = os.path.join(folder_path, 'config.txt')
    kinematics_file = os.path.join(folder_path, 'kinematics.txt')
    dynamics_file = os.path.join(folder_path, 'dynamics.txt')

    # Read variable names from config
    kinematics_vars, dynamics_vars = read_config(config_file)

    # Read data from kinematics and dynamics files
    kinematics_data = read_data(kinematics_file, kinematics_vars)
    dynamics_data = read_data(dynamics_file, dynamics_vars)

    PD_2 = np.mean(np.abs(dynamics_data['PD'][:, 0]))
    PD_4 = np.mean(np.abs(dynamics_data['PD'][:, 2]))
    print("PD_2: ", PD_2)
    print("PD_4: ", PD_4)

    # cal_extorq = (dynamics_data[dynamics_vars[4]] - dynamics_data[dynamics_vars[3]] - (dynamics_data[dynamics_vars[1]] - dynamics_data[dynamics_vars[2]] - dynamics_data[dynamics_vars[3]] - dynamics_data[dynamics_vars[5]]))
    # cal_extorq = dynamics_data[dynamics_vars[3]] - dynamics_data[dynamics_vars[6]] - dynamics_data[dynamics_vars[4]]
    cal_extorq = dynamics_data['inverse dynamic'] - dynamics_data['dynamic feed forward']- (dynamics_data['PD']) 
    dynamics_data['calculated external torque'] = cal_extorq
    dynamics_vars.append("calculated external torque")

    # Impedance Calculation and Appending
    velocity_threshold = 0.01  # Define a threshold for velocity
    min_len = len(kinematics_data['feedback joint vel'])  # Use the minimum length to match time ranges
    valid_indices = np.abs(kinematics_data['feedback joint vel'][:min_len, :]) > velocity_threshold
    impedance_data = np.where(valid_indices, dynamics_data['applied external torque'][:min_len, :] / (kinematics_data['feedback joint vel'][:min_len, :] + 1e-6), np.nan)
    dynamics_data['impedance'] = impedance_data
    dynamics_vars.append("impedance")

    num_data = 3 + len(dynamics_vars) - 1
    print(num_data)
    
    plot_data(kinematics_data, dynamics_data, num_data)
    calculate_errors(kinematics_data, dynamics_data)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <folder_name>")
    else:
        main(sys.argv[1])
