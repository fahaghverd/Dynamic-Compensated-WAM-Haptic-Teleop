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
    
    pos_des_2 = kinematics_data['desired joint pos'][:, 0]
    pos_feedback_2 = kinematics_data['feedback joint pos'][:, 0]
    pos_nrmse_2 = calculate_nrmse(pos_des_2, pos_feedback_2)

    pos_des_4 = kinematics_data['desired joint pos'][:, 2]
    pos_feedback_4 = kinematics_data['feedback joint pos'][:, 2]
    pos_nrmse_4 = calculate_nrmse(pos_des_4, pos_feedback_4)

    extorq = dynamics_data['wam joint torque input'] - dynamics_data['wam gravity input'] - dynamics_data['inverse dynamic']
    extorq_2 = extorq[:, 0]
    extorq_4 = extorq[:, 2]