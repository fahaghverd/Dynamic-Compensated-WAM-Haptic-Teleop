import matplotlib.pyplot as plt
import numpy as np
import argparse
from scipy.signal import butter, filtfilt

# Function to create a low-pass Butterworth filter
def butter_lowpass(cutoff=10, fs=500, order=4):
    nyquist = 0.5 * fs  # Nyquist Frequency
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

# Function to apply the low-pass filter
def lowpass_filter(data):
    b, a = butter_lowpass()
    y = filtfilt(b, a, data)  # Apply filter with zero-phase filtering
    return y

parser = argparse.ArgumentParser()
parser.add_argument("file_name")
args = parser.parse_args()


# Assuming the data is stored in a text file named 'output.txt'
file_path_config = "bin/" + args.file_name + "_config"
file_path_jt = "bin/" + args.file_name + "_jt"

# Lists to hold the parsed data for only the 2nd and 4th joints
time_log = []
jp_current_2 = []
jp_desired_2 = []
jp_current_4 = []
jp_desired_4 = []
jv_current_2 = []
jv_desired_2 = []
jv_current_4 = []
jv_desired_4 = []
ja_current_2 = []
ja_desired_2 = []
ja_current_4 = []
ja_desired_4 = []

time_log_jt = []
jt_sum_2 = []
jt_sum_4 = []
gravity_2 = []
gravity_4 = []
dynamics_feed_fwd_2 = []
dynamics_feed_fwd_4 = []
invdyn_2 = []
invdyn_4 = []
# applied_tauh_2 = []
# applied_tauh_4 = []

frq = 500
# Reading and parsing the data from the file
with open(file_path_config, 'r') as file:
    # for line in file:
    for i, line in enumerate(file):
        # if i >= 30*frq and i <= 37.5*frq:
        # Assuming the tuple is written in a format like: 
        # (time, jp_current, jp_desired, jt_sum, gravity, control_jt, dynamics_feed_fwd)
            data = eval(line.strip())
            time_log.append(data[0])
            # Append the data for the 2nd and 4th joints
            jp_desired_2.append(data[2])  # 2nd joint (index 1)
            jp_desired_4.append(data[4])  # 4th joint (index 3)
            jp_current_2.append(data[9])  # 2nd joint (index 1)
            jp_current_4.append(data[11])  # 4th joint (index 3)

            jv_desired_2.append(data[16])  # 2nd joint (index 1)
            jv_desired_4.append(data[18])  # 4th joint (index 3)
            jv_current_2.append(data[23])  # 2nd joint (index 1)
            jv_current_4.append(data[25])  # 4th joint (index 3)

            ja_desired_2.append(data[30])  # 2nd joint (index 1)
            ja_desired_4.append(data[32])  # 4th joint (index 3)
            ja_current_2.append(data[37])  # 2nd joint (index 1)
            ja_current_4.append(data[39])  # 4th joint (index 3)

with open(file_path_jt, 'r') as file:
    # for line in file:
    for i, line in enumerate(file):
        # if i >= 30*frq and i <= 37.5*frq:
        # Assuming the tuple is written in a format like: 
        # (time, jp_current, jp_desired, jt_sum, gravity, control_jt, dynamics_feed_fwd)
            data = eval(line.strip())
            time_log_jt.append(data[0])
            jt_sum_2.append(data[2])
            jt_sum_4.append(data[4])
            gravity_2.append(data[9])
            gravity_4.append(data[11])
            dynamics_feed_fwd_2.append(data[16])
            dynamics_feed_fwd_4.append(data[18])
            invdyn_2.append(data[23])
            invdyn_4.append(data[25])
            # applied_tauh_2.append(data[13])
            # applied_tauh_4.append(data[15])


# Convert lists to numpy arrays for easier handling
time_log = np.array(time_log)
jp_current_2 = np.array(jp_current_2)
jp_desired_2 = np.array(jp_desired_2)
jp_current_4 = np.array(jp_current_4)
jp_desired_4 = np.array(jp_desired_4)

jv_current_2 = np.array(jv_current_2)
jv_desired_2 = np.array(jv_desired_2)
jv_current_4 = np.array(jv_current_4)
jv_desired_4 = np.array(jv_desired_4)

ja_current_2 = np.array(ja_current_2)
ja_desired_2 = np.array(ja_desired_2)
ja_current_4 = np.array(ja_current_4)
ja_desired_4 = np.array(ja_desired_4)

time_log_jt = np.array(time_log_jt)
jt_sum_2 = np.array(jt_sum_2)
jt_sum_4 = np.array(jt_sum_4)

gravity_2 = np.array(gravity_2)
gravity_4 = np.array(gravity_4)

dynamics_feed_fwd_2 = np.array(dynamics_feed_fwd_2)
dynamics_feed_fwd_4 = np.array(dynamics_feed_fwd_4)

invdyn_2 = np.array(invdyn_2)
invdyn_4 = np.array(invdyn_4)

# pos_error_2 = (jp_desired_2 - jp_current_2)/jp_desired_2
# pos_error_4 = (jp_desired_4 - jp_current_4)/jp_desired_4
# print("error_2 mean abs = ", np.mean(np.abs(pos_error_2)))
# print("error_4 mean abs = ", np.mean(np.abs(pos_error_4)))

# pd_term_2 =(jt_sum_2-(gravity_2 + dynamics_feed_fwd_2))
# pd_term_4 =(jt_sum_4-(gravity_4 + dynamics_feed_fwd_4))
# print("pd_term_2 mean abs = ", np.mean(np.abs(pd_term_2)))
# print("pd_term_4 mean abs = ", np.mean(np.abs(pd_term_4)))

# print("torque 2 mean abs:", np.mean(np.abs(jt_sum_2)))
# print("torque 4 mean abs:", np.mean(np.abs(jt_sum_4)))
# print("dynamics_feed_fwd_2 :", np.mean(dynamics_feed_fwd_2))
# print("dynamics_feed_fwd_4 4:", np.mean(dynamics_feed_fwd_4))


cal_tauh_2 = jt_sum_2 - gravity_2 - invdyn_2
cal_tauh_4 = jt_sum_4 - gravity_4 - invdyn_4
print("Cal Tauh 2: ", np.mean(np.abs(cal_tauh_2)))
print("Cal Tauh 4: ", np.mean(np.abs(cal_tauh_4)))


filt_cal_tauh_2 = lowpass_filter(cal_tauh_2)
filt_cal_tauh_4 = lowpass_filter(cal_tauh_4)

filt_dynamics_feed_fwd_2= lowpass_filter(dynamics_feed_fwd_2)
filt_dynamics_feed_fwd_4 = lowpass_filter(dynamics_feed_fwd_4)

filt_invdyn_2= lowpass_filter(invdyn_2)
filt_invdyn_4 = lowpass_filter(invdyn_4)

cal_tauh_2_filt_id = jt_sum_2 - gravity_2 - filt_invdyn_2
cal_tauh_4_filt_id = jt_sum_4 - gravity_4 - filt_invdyn_4


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

num = 8
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

plt.subplot(num, 2, 3)
plt.plot(time_log, jv_current_2, label='Velocity 2nd Joint')
plt.plot(time_log, jv_desired_2, label='Desired Velocity 2nd Joint')
plt.xlabel('Time')
plt.ylabel('Velocity 2nd Joint')
plt.legend()

# Plot for the 4th joint position
plt.subplot(num, 2, 4)
plt.plot(time_log, jv_current_4, label='Velocity 4th Joint')
plt.plot(time_log, jv_desired_4, label='Desired Velocity 4th Joint')
plt.xlabel('Time')
plt.ylabel('Velocity 4th Joint')
plt.legend()

plt.subplot(num, 2, 5)
plt.plot(time_log, ja_current_2, label='Acceleration 2nd Joint')
plt.plot(time_log, ja_desired_2, label='Desired Acceleration 2nd Joint')
plt.xlabel('Time')
plt.ylabel('Acceleration 2nd Joint')
plt.legend()

# Plot for the 4th joint position
plt.subplot(num, 2, 6)
plt.plot(time_log, ja_current_4, label='Acceleration 4th Joint')
plt.plot(time_log, ja_desired_4, label='Desired Acceleration 4th Joint')
plt.xlabel('Time')
plt.ylabel('Acceleration 4th Joint')
plt.legend()

# plt.subplot(num, 2, 3)
# plt.plot(time_log, error_2, label='Nom. Position Error 2th Joint')
# plt.xlabel('Time')
# plt.ylabel('Nom. Position Error 2th Joint')
# plt.legend()

# plt.subplot(num, 2, 4)
# plt.plot(time_log, error_4, label='Nom. Position Error 4th Joint')
# plt.xlabel('Time')
# plt.ylabel('Nom. Position Error 4th Joint')
# plt.legend()

# Plot for the 2nd joint torque sum
plt.subplot(num, 2, 7)
plt.plot(time_log_jt, jt_sum_2, label='Torque Sum 2nd Joint')
plt.xlabel('Time')
plt.ylabel('Torque Sum 2nd Joint')
plt.legend()

# Plot for the 4th joint torque sum
plt.subplot(num, 2, 8)
plt.plot(time_log_jt, jt_sum_4, label='Torque Sum 4th Joint')
plt.xlabel('Time')
plt.ylabel('Torque Sum 4th Joint')
plt.legend()

# # Plot for the 2nd joint torque sum
# plt.subplot(num, 2, 7)
# plt.plot(time_log, pd_term_2, label='PID 2nd Joint')
# plt.xlabel('Time')
# plt.ylabel('PID 2nd Joint')
# plt.legend()

# # Plot for the 4th joint torque sum
# plt.subplot(num, 2, 8)
# plt.plot(time_log, pd_term_4, label='PID 4th Joint')
# plt.xlabel('Time')
# plt.ylabel('PID 4th Joint')
# plt.legend()

# Plot for the 2nd joint gravity
plt.subplot(num, 2, 9)
plt.plot(time_log_jt, gravity_2, label='Gravity 2nd Joint')
plt.xlabel('Time')
plt.ylabel('Gravity 2nd Joint')
plt.legend()

# Plot for the 4th joint gravity
plt.subplot(num, 2, 10)
plt.plot(time_log_jt, gravity_4, label='Gravity 4th Joint')
plt.xlabel('Time')
plt.ylabel('Gravity 4th Joint')
plt.legend()

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
plt.subplot(num, 2, 11)
plt.plot(time_log_jt, dynamics_feed_fwd_2, label='Dynamics Feed Fwd 2nd Joint')
plt.plot(time_log_jt, filt_dynamics_feed_fwd_2, label='Filt Dynamics Feed Fwd 2nd Joint')
plt.xlabel('Time')
plt.ylabel('Dynamics Feed Fwd 2nd Joint')
plt.legend()

# Plot for the 4th joint dynamics feed forward
plt.subplot(num, 2, 12)
plt.plot(time_log_jt, dynamics_feed_fwd_4, label='Dynamics Feed Fwd 4th Joint')
plt.plot(time_log_jt, filt_dynamics_feed_fwd_4, label='Filt Dynamics Feed Fwd 4th Joint')
plt.xlabel('Time')
plt.ylabel('Dynamics Feed Fwd 4th Joint')
plt.legend()

plt.subplot(num, 2, 13)
plt.plot(time_log_jt, invdyn_2, label='Inv Dynamics 2nd Joint')
plt.plot(time_log_jt, filt_invdyn_2, label='Filt Inv Dynamics 2nd Joint')
plt.xlabel('Time')
plt.ylabel('Inv Dynamics 2nd Joint')
plt.legend()

# Plot for the 4th joint dynamics feed forward
plt.subplot(num, 2, 14)
plt.plot(time_log_jt, invdyn_4, label='Inv Dynamics 4th Joint')
plt.plot(time_log_jt, filt_invdyn_4, label='Filt Inv Dynamics 4th Joint')
plt.xlabel('Time')
plt.ylabel('Inv Dynamics 4th Joint')
plt.legend()

plt.subplot(num, 2, 15)
# plt.plot(time_log_jt, applied_tauh_2, label='Applied Tauh 2nd Joint')
plt.plot(time_log_jt, cal_tauh_2, label='Cal Tauh 2nd Joint')
plt.plot(time_log_jt, filt_cal_tauh_2, label='Filt Cal Tauh 2nd Joint')
plt.plot(time_log_jt, cal_tauh_2_filt_id, label='Cal tauh Filt ID 2nd Joint')
plt.xlabel('Time')
plt.ylabel('Tauh 2nd Joint')
plt.legend()

plt.subplot(num, 2, 16)
# plt.plot(time_log_jt, applied_tauh_4, label='Applied Tauh 4nd Joint')
plt.plot(time_log_jt, cal_tauh_4, label='Cal Tauh 4nd Joint')
plt.plot(time_log_jt, filt_cal_tauh_4, label='Filt Cal Tauh 4th Joint')
plt.plot(time_log_jt, cal_tauh_4_filt_id, label='Cal tauh Filt ID 4th Joint')
plt.xlabel('Time')
plt.ylabel('Tauh 4nd Joint')
plt.legend()

plt.tight_layout()
plt.show()


