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
                    idx = (i - 1) * 7 + 1  # Adjust index for the rest of the variables
                    if idx < len(values):
                        data_dict[variable_names[i]].append([
                            float(values[idx]),
                            float(values[idx + 1]) if idx + 1 < len(values) else None,
                            float(values[idx + 2]) if idx + 2 < len(values) else None,
                            float(values[idx + 3]) if idx + 3 < len(values) else None,
                            float(values[idx + 4]) if idx + 4 < len(values) else None,
                            float(values[idx + 5]) if idx + 5 < len(values) else None,
                            float(values[idx + 6]) if idx + 6 < len(values) else None
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
    plt.plot(dynamics_time, dynamics_data['inverse dynamic'][:, 1])
    plt.title('Joint 2 WAM Inverse Dynamic')
    plt.xlabel('Time (s)')
    plt.ylabel('Inverse Dynamic')

    plt.subplot(num_data, 2, 12)
    plt.plot(dynamics_time, dynamics_data['inverse dynamic'][:, 3])
    plt.title('Joint 4 WAM Inverse Dynamic')
    plt.xlabel('Time (s)')
    plt.ylabel('Inverse Dynamic')


    plt.subplot(num_data, 2, 13)
    plt.plot(dynamics_time, dynamics_data['calculated external torque'][:, 1])
    plt.title('Joint 2 External Torque')
    plt.xlabel('Time (s)')
    plt.ylabel('External Torque')
    plt.legend()

    plt.subplot(num_data, 2, 14)
    plt.plot(dynamics_time, dynamics_data['calculated external torque'][:, 3])
    plt.title('Joint 4 External Torque')
    plt.xlabel('Time (s)')
    plt.ylabel('External Torque')
    plt.legend()


    plt.subplot(num_data, 2, 15)
    plt.plot(dynamics_time, dynamics_data['PD'][:, 1], label='Joint 2 PD', linestyle='-')
    plt.title('Joint 2')
    plt.xlabel('Time (s)')
    plt.ylabel('PD')
    plt.legend()

    plt.subplot(num_data, 2, 16)
    plt.plot(dynamics_time, dynamics_data['PD'][:, 3], label='PD', linestyle='-')
    plt.title('PD')
    plt.xlabel('Time (s)')
    plt.ylabel('PD')
    plt.legend()

    plt.show()


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

    cal_extorq = dynamics_data['inverse dynamic'] - dynamics_data['PD']
    dynamics_data['calculated external torque'] = cal_extorq
    dynamics_vars.append("calculated external torque")

    num_data = 3 + len(dynamics_vars) - 1

    plot_data(kinematics_data, dynamics_data, num_data)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <folder_name>")
    else:
        main(sys.argv[1])
