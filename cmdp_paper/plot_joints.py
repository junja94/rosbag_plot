import csv
import matplotlib.pyplot as plt
import math
import numpy as np
import glob
from scipy.signal import butter, lfilter, freqz
import ast

# total_time = 2.5 # 2.0 sideways
# total_time = 2.0
total_time = 3.0
n_duration = int(total_time / 0.0025)
print(n_duration)
# n_duration = -1

# csv_path = ("/home/jolee/git/rosbag_plot/cmdp_paper/data/0/motion_400hz.csv", 1000, n_duration )
# csv_path = ("/home/jolee/git/rosbag_plot/cmdp_paper/data/cmdp_step_1/motion_400hz.csv", 0, n_duration )
# csv_path = ("/home/jolee/git/rosbag_plot/cmdp_paper/data/cmdp_step_2/motion_400hz.csv", 1150, n_duration )
# csv_path = ("/home/jolee/git/rosbag_plot/cmdp_paper/data/cmdp_side_1/motion_400hz.csv", 3000, n_duration )
# csv_path = ("/home/jolee/git/rosbag_plot/cmdp_paper/data/ppo_step_1/motion_400hz.csv", 0, n_duration )
# csv_path = ("/home/jolee/git/rosbag_plot/cmdp_paper/data/ppo_step_2/motion_400hz.csv", 21600, n_duration )
# csv_path = ("/home/jolee/git/rosbag_plot/cmdp_paper/data/ppo_side_1/motion_400hz.csv", 23600, n_duration )

csv_path = ("/home/jolee/git/rosbag_plot/cmdp_paper/data/ppo_side_1/motion_400hz.csv", 22800, n_duration )
# csv_path = ("/home/jolee/git/rosbag_plot/cmdp_paper/data/cmdp_side_1/motion_400hz.csv", 2900, n_duration )

num_joint = 16

#IBM color blind safe pallette
color_list = ["#648fff", "#785ef0", "#dc267f", "#fe6100", "#ffb000", "#000000", "#ffffff"]


robot_mass = 63.8975

timestamps = []
moving_speeds = []
joint_velocities = []
joint_torques = []

with open(csv_path[0], mode='r') as csv_file:
    reader = csv.DictReader(csv_file)
    rows = list(reader)  # Convert reader object to a list of rows
    max_index = len(rows) - 1  #
    max_index_to_plot = min(len(rows) - 1, csv_path[1] + csv_path[2])
    if csv_path[2] == -1:
        max_index_to_plot = len(rows) - 1

    for j, row in enumerate(rows):
        if j < csv_path[1]:
            continue
        if j > max_index_to_plot:
            break

        t = float(row['time'])
        linear_x = float(row['linear_x'])
        linear_y = float(row['linear_y'])
        linear_z = float(row['linear_z'])

        joint_tor = ast.literal_eval(row['joint_torque'])
        joint_vel = ast.literal_eval(row['joint_velocity'])

        leg_vel = []
        leg_tor = []
        for i in range(4):
            leg_vel.append(joint_vel[4*i:4*i+3])
            leg_tor.append(joint_tor[4*i:4*i+3])

        leg_vel = np.concatenate(leg_vel)
        leg_tor = np.concatenate(leg_tor)

        # hor_vel = np.array([linear_x, linear_y])
        # hor_vel = np.array([linear_x, 0.0])
        hor_vel = np.array([0.0, linear_y])
        hor_speed = np.linalg.norm(hor_vel)

        moving_speeds.append(hor_speed)
        joint_velocities.append(leg_vel)
        joint_torques.append(leg_tor)
        timestamps.append(t)
        # print(hor_speed)

print(moving_speeds)
moving_speeds = np.stack(moving_speeds)
joint_velocities = np.stack(joint_velocities, axis=0)
joint_torques = np.stack(joint_torques, axis=0)

print(joint_velocities.shape)

# fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(4.0, 5.0))
fig, axes = plt.subplots(nrows=3, ncols=1)

axes[0].plot(joint_velocities, color=color_list[0])
axes[0].plot([0.0, len(joint_torques)], [6.0, 6.0], color=color_list[2], linestyle='--')
axes[0].plot([0.0, len(joint_torques)], [-6.0, -6.0], color=color_list[2], linestyle='--')

axes[1].plot(joint_torques, color=color_list[1])
axes[1].plot([0.0, len(joint_torques)], [75.0, 75.0], color=color_list[2], linestyle='--')
axes[1].plot([0.0, len(joint_torques)], [-75.0, -75.0], color=color_list[2], linestyle='--')

axes[2].plot(moving_speeds, color=color_list[2])
fig.tight_layout()

timestamps = np.array(timestamps)
timestamps = timestamps - timestamps[0]
timestamps = np.linspace(0, 0.0025 * (len(timestamps) - 1), len(timestamps))

label_size = 7

# fig, ax = plt.subplots(nrows=3, ncols=1, figsize=(2.75, 3.0))
fig, ax = plt.subplots(nrows=3, ncols=1, figsize=(2.5, 3.0))
ax[1].plot(timestamps, joint_velocities, color=color_list[0])
ax[1].plot([0.0, timestamps[-1]], [6.0, 6.0], color=color_list[2], linestyle='--')
ax[1].plot([0.0, timestamps[-1]], [-6.0, -6.0], color=color_list[2], linestyle='--')
ax[1].set_yticks([ -8.0, -4.0, 0, 4.0, 8.0])
ax[1].set_ylim([ -9.0, 9.0])

ax[2].plot(timestamps, joint_torques, color=color_list[0])
ax[2].plot([0.0, timestamps[-1]], [75.0, 75.0], color=color_list[2], linestyle='--')
ax[2].plot([0.0, timestamps[-1]], [-75.0, -75.0], color=color_list[2], linestyle='--')
ax[2].set_yticks([-100, -50, 0, 50, 100])
ax[2].set_ylim([ -100.0, 100.0])

ax[0].plot(timestamps, moving_speeds, color=color_list[0])
ax[0].plot([0.0, timestamps[-1]], [1.0, 1.0], color=color_list[3], linestyle='-')
ax[0].set_yticks([ 0.0, 0.5, 1.0])
ax[0].set_ylim([ 0.0, 1.3])

print(np.mean(1.0 - moving_speeds))
print(np.std(1.0 - moving_speeds))

ax[0].set_xticks([ ])
# ax[1].set_xticks([ ])

if n_duration != -1:
    for axis in ax:
        axis.tick_params(axis='both', labelsize=7)
        axis.set_xlim([0.0, total_time])


# axes[2].tick_params(axis='both', labelsize=8)

fig.tight_layout()
plt.savefig('saved_images/joints.svg')

plt.show()
