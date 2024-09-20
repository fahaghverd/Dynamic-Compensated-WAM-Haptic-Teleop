import matplotlib.pyplot as plt
import numpy as np
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("file_name")
args = parser.parse_args()


# Assuming the data is stored in a text file named 'output.txt'
file_path = args.file_name

# Lists to hold the parsed data for only the 2nd and 4th joints
time_log = []
jp_current_2 = []
jp_desired_2 = []
jp_current_4 = []
jp_desired_4 = []
jt_sum_2 = []
jt_sum_4 = []
gravity_2 = []
gravity_4 = []
# control_jt_2 = []
# control_jt_4 = []
# dynamics_feed_fwd_2 = []
# dynamics_feed_fwd_4 = []
jv_desired_2 = []
jv_desired_4 = []
# ja_desired_2 = []
# ja_desired_4 = []

# Reading and parsing the data from the file
with open(file_path, 'r') as file:
    for line in file:
        # Assuming the tuple is written in a format like: 
        # (time, jp_current, jp_desired, jt_sum, gravity, control_jt, dynamics_feed_fwd)
        data = eval(line.strip())
        time_log.append(data[0])
        # Append the data for the 2nd and 4th joints
        jp_current_2.append(data[9])  # 2nd joint (index 1)
        jp_desired_2.append(data[2])  # 2nd joint (index 1)
        jp_current_4.append(data[11])  # 4th joint (index 3)
        jp_desired_4.append(data[4])  # 4th joint (index 3)
        jt_sum_2.append(data[16])
        jt_sum_4.append(data[18])
        gravity_2.append(data[23])
        gravity_4.append(data[25])
        # control_jt_2.append(data[30])
        # control_jt_4.append(data[32])
        # dynamics_feed_fwd_2.append(data[30])
        # # dynamics_feed_fwd_4.append(data[32])
        # jv_desired_2.append(data[30])
        # jv_desired_4.append(data[32])
        # ja_desired_2.append(data[44])
        # ja_desired_4.append(data[46])

# Convert lists to numpy arrays for easier handling
time_log = np.array(time_log)
jp_current_2 = np.array(jp_current_2)
jp_desired_2 = np.array(jp_desired_2)
jp_current_4 = np.array(jp_current_4)
jp_desired_4 = np.array(jp_desired_4)
# position_error_2 = jp_current_2 - jp_desired_2
# position_error_4 = jp_current_4 - jp_desired_4
jt_sum_2 = np.array(jt_sum_2)
jt_sum_4 = np.array(jt_sum_4)
gravity_2 = np.array(gravity_2)
gravity_4 = np.array(gravity_4)
error_2 = (jp_desired_2 - jp_current_2)/jp_desired_2
error_4 = (jp_desired_4 - jp_current_4)/jp_desired_4
pd_term_2 = jt_sum_2 - gravity_2
pd_term_4 = jt_sum_4 - gravity_4
print("error_2 mean abs = ", np.mean(np.abs(error_2)))
print("error_4 mean abs = ", np.mean(np.abs(error_4)))
print("pd_term_2 mean = ", np.mean(np.abs(pd_term_2)))
print("pd_term_4 mean = ", np.mean(np.abs(pd_term_4)))
# control_jt_2 = np.array(control_jt_2)
# control_jt_4 = np.array(control_jt_4)
# dynamics_feed_fwd_2 = np.array(dynamics_feed_fwd_2)
# dynamics_feed_fwd_4 = np.array(dynamics_feed_fwd_4)
# jv_desired_2 = np.array(jv_desired_2)
# jv_desired_4 = np.array(jv_desired_4)
# ja_desired_2 = np.array(ja_desired_2)
# ja_desired_4 = np.array(ja_desired_4)
# # Plotting the data
# plt.figure(figsize=(14, 10))

# # Plot for the 2nd joint
# plt.subplot(2, 1, 1)
# plt.plot(time_log, jp_current_2, label='Position 2nd Joint')
# plt.plot(time_log, jp_desired_2, label='Desired Position 2nd Joint')
# plt.xlabel('Time')
# plt.ylabel('Position 2nd Joint')
# plt.legend()

# # Plot for the 4th joint
# plt.subplot(2, 1, 2)
# plt.plot(time_log, jp_current_4, label='Position 4th Joint')
# plt.plot(time_log, jp_desired_4, label='Desired Position 4th Joint')
# plt.xlabel('Time')
# plt.ylabel('Position 4th Joint')
# plt.legend()

# plt.tight_layout()
# plt.show()

# Plotting the data
plt.figure(figsize=(14, 20))

num = 5

# Plot for the 2nd joint position
plt.subplot(num, 2, 1)
plt.plot(time_log, jp_current_2, label='Position 2nd Joint')
plt.plot(time_log, jp_desired_2, label='Desired Position 2nd Joint')
plt.xlabel('Time')
plt.ylabel('Position 2nd Joint')
plt.legend()

# Plot for the 4th joint position
plt.subplot(num, 2, 2)
plt.plot(time_log, jp_current_4, label='Position 4th Joint')
plt.plot(time_log, jp_desired_4, label='Desired Position 4th Joint')
plt.xlabel('Time')
plt.ylabel('Position 4th Joint')
plt.legend()

# Plot for the 4th joint dynamics feed forward
plt.subplot(num, 2, 3)
plt.plot(time_log, error_2, label='Nom. Position Error 2th Joint')
plt.xlabel('Time')
plt.ylabel('Error 2th Joint')
plt.legend()

plt.subplot(num, 2, 4)
plt.plot(time_log, error_4, label='Nom. Position Error 4th Joint')
plt.xlabel('Time')
plt.ylabel('Error 4th Joint')
plt.legend()

# Plot for the 2nd joint torque sum
plt.subplot(num, 2, 5)
plt.plot(time_log, jt_sum_2, label='Torque Sum 2nd Joint')
plt.xlabel('Time')
plt.ylabel('Torque Sum 2nd Joint')
plt.legend()

# Plot for the 4th joint torque sum
plt.subplot(num, 2, 6)
plt.plot(time_log, jt_sum_4, label='Torque Sum 4th Joint')
plt.xlabel('Time')
plt.ylabel('Torque Sum 4th Joint')
plt.legend()

# Plot for the 2nd joint torque sum
plt.subplot(num, 2, 7)
plt.plot(time_log, pd_term_2, label='PID 2nd Joint')
plt.xlabel('Time')
plt.ylabel('PID 2nd Joint')
plt.legend()

# Plot for the 4th joint torque sum
plt.subplot(num, 2, 8)
plt.plot(time_log, pd_term_4, label='PID 4th Joint')
plt.xlabel('Time')
plt.ylabel('PID 4th Joint')
plt.legend()

# Plot for the 2nd joint gravity
plt.subplot(num, 2, 9)
plt.plot(time_log, gravity_2, label='Gravity 2nd Joint')
plt.xlabel('Time')
plt.ylabel('Gravity 2nd Joint')
plt.legend()

# Plot for the 4th joint gravity
plt.subplot(num, 2, 10)
plt.plot(time_log, gravity_4, label='Gravity 4th Joint')
plt.xlabel('Time')
plt.ylabel('Gravity 4th Joint')
plt.legend()

# # Plot for the 4th joint dynamics feed forward
# plt.subplot(4, 2, 11)
# plt.plot(time_log, error_2, label='Nom. Position Error 2th Joint')
# plt.xlabel('Time')
# plt.ylabel('Error 2th Joint')
# plt.legend()

# plt.subplot(4, 2, 12)
# plt.plot(time_log, error_4, label='Nom. Position Error 4th Joint')
# plt.xlabel('Time')
# plt.ylabel('Error 4th Joint')
# plt.legend()
# # Plot for the 2nd joint control torque
# plt.subplot(6, 2, 7)
# plt.plot(time_log, control_jt_2, label='Control Torque 2nd Joint')
# plt.xlabel('Time')
# plt.ylabel('Control Torque 2nd Joint')
# plt.legend()

# # Plot for the 4th joint control torque
# plt.subplot(6, 2, 8)
# plt.plot(time_log, control_jt_4, label='Control Torque 4th Joint')
# plt.xlabel('Time')
# plt.ylabel('Control Torque 4th Joint')
# plt.legend()

# Plot for the 2nd joint dynamics feed forward
# plt.subplot(6, 2, 7)
# plt.plot(time_log, dynamics_feed_fwd_2, label='Dynamics Feed Fwd 2nd Joint')
# plt.xlabel('Time')
# plt.ylabel('Dynamics Feed Fwd 2nd Joint')
# plt.legend()

# # Plot for the 4th joint dynamics feed forward
# plt.subplot(6, 2, 8)
# plt.plot(time_log, dynamics_feed_fwd_4, label='Dynamics Feed Fwd 4th Joint')
# plt.xlabel('Time')
# plt.ylabel('Dynamics Feed Fwd 4th Joint')
# plt.legend()

# Plot for the 4th joint dynamics feed forward
# plt.subplot(6, 2, 9)
# plt.plot(time_log, jv_desired_2, label='Des Vel 2th Joint')
# plt.xlabel('Time')
# plt.ylabel('Des Vel 2th Joint')
# plt.legend()

# plt.subplot(6, 2, 10)
# plt.plot(time_log, jv_desired_4, label='Des Vel 4th Joint')
# plt.xlabel('Time')
# plt.ylabel('Des Vel 4th Joint')
# plt.legend()

# Plot for the 4th joint dynamics feed forward
# plt.subplot(6, 2, 11)
# plt.plot(time_log, ja_desired_2, label='Des Acc 2th Joint')
# plt.xlabel('Time')
# plt.ylabel('Des Acc 2th Joint')
# plt.legend()

# plt.subplot(6, 2, 12)
# plt.plot(time_log, ja_desired_4, label='Des Acc 4th Joint')
# plt.xlabel('Time')
# plt.ylabel('Des Acc 4th Joint')
plt.legend()



plt.tight_layout()
plt.show()


