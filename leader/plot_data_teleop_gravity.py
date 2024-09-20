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

def plot_kinematics_data(time, data, title):
    plt.figure(figsize=(12, 12))

    # Subplot for Joint 2 Position
    plt.subplot(3, 2, 1)
    plt.plot(time, data['desired joint pos'][:, 0], label='Desired Joint 2 Position', linestyle='--')
    plt.plot(time, data['feedback joint pos'][:, 0], label='Feedback Joint 2 Position', linestyle='-')
    plt.title('Joint 2 Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend()

    # Subplot for Joint 4 Position
    plt.subplot(3, 2, 2)
    plt.plot(time, data['desired joint pos'][:, 2], label='Desired Joint 4 Position', linestyle='--')
    plt.plot(time, data['feedback joint pos'][:, 2], label='Feedback Joint 4 Position', linestyle='-')
    plt.title('Joint 4 Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend()

    # Subplot for Joint 2 Velocity
    plt.subplot(3, 2, 3)
    plt.plot(time, data['desired joint vel'][:, 0], label='Desired Joint 2 Velocity', linestyle='--')
    plt.plot(time, data['feedback joint vel'][:, 0], label='Feedback Joint 2 Velocity', linestyle='-')
    plt.title('Joint 2 Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.legend()

    # Subplot for Joint 4 Velocity
    plt.subplot(3, 2, 4)
    plt.plot(time, data['desired joint vel'][:, 2], label='Desired Joint 4 Velocity', linestyle='--')
    plt.plot(time, data['feedback joint vel'][:, 2], label='Feedback Joint 4 Velocity', linestyle='-')
    plt.title('Joint 4 Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.legend()

    # Subplot for Joint 2 Acceleration
    plt.subplot(3, 2, 5)
    plt.plot(time, data['desired joint acc'][:, 0], label='Desired Joint 2 Acceleration', linestyle='--')
    plt.plot(time, data['feedback joint acc'][:, 0], label='Feedback Joint 2 Acceleration', linestyle='-')
    plt.title('Joint 2 Acceleration')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration')
    plt.legend()

    # Subplot for Joint 4 Acceleration
    plt.subplot(3, 2, 6)
    plt.plot(time, data['desired joint acc'][:, 2], label='Desired Joint 4 Acceleration', linestyle='--')
    plt.plot(time, data['feedback joint acc'][:, 2], label='Feedback Joint 4 Acceleration', linestyle='-')
    plt.title('Joint 4 Acceleration')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration')
    plt.legend()

    plt.tight_layout()
    plt.suptitle(title, fontsize=16, y=1.02)
    plt.show()

def plot_dynamics_data(time, data, variable_names, title):
    num_vars = len(variable_names) - 1  # Exclude time
    plt.figure(figsize=(12, 3 * num_vars))

    for i, var in enumerate(variable_names[1:]):  # Skip time
        first_col = data[var][:, 0]  # Joint 2
        third_col = data[var][:, 2]  # Joint 4
        
        plt.subplot(num_vars, 2, i * 2 + 1)  # First column (Joint 2)
        plt.plot(time, first_col)  # Simplified without line style and marker
        plt.title(f"{var} - Joint 2")
        plt.xlabel("Time (s)")
        plt.ylabel("Value")
        
        plt.subplot(num_vars, 2, i * 2 + 2)  # Second column (Joint 4)
        plt.plot(time, third_col)  # Simplified without line style and marker
        plt.title(f"{var} - Joint 4")
        plt.xlabel("Time (s)")
        plt.ylabel("Value")

    plt.tight_layout()
    plt.suptitle(title, fontsize=16, y=1.02)
    plt.show()

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

    # Plot kinematics data
    if kinematics_data:
        plot_kinematics_data(kinematics_data[kinematics_vars[0]], kinematics_data, "Kinematics Data vs Time")

    # Plot dynamics data
    if dynamics_data:
        plot_dynamics_data(dynamics_data[dynamics_vars[0]], dynamics_data, dynamics_vars, "Dynamics Data vs Time")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <folder_name>")
    else:
        main(sys.argv[1])
