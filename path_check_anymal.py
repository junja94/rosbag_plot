import csv
import matplotlib.pyplot as plt
import math
import numpy as np
import glob
from scipy.signal import butter, lfilter, freqz


csv_paths = [
    # ("data/0522_mission/motion.csv", 0, 0),
    ("data/anymal_c/motion.csv", 0, 0),
]

# color_list = ["#fd7f6f", "#7eb0d5", "#b2e061", "#bd7ebe", "#ffb55a", "#ffee65", "#beb9db", "#fdcce5", "#8bd3c7"]

#IBM color blind safe pallette
color_list = ["#648fff", "#785ef0", "#dc267f", "#fe6100", "#ffb000", "#000000", "#ffffff"]
#dc267f
#fe6100
#ffb000
#000000
#ffffff

robot_mass = 51.3

all_time_stamps = []
all_moving_speeds = []
all_base_positions = []
all_cot = []

for i, csv_path in enumerate(csv_paths):
    timestamps = []
    moving_speeds = []
    base_positions = []
    joint_velocities = []
    joint_torques = []
    cots_recorded = []

    with open(csv_path[0], mode='r') as csv_file:
        reader = csv.DictReader(csv_file)
        rows = list(reader)  # Convert reader object to a list of rows
        max_index = len(rows) - 1  #
        print(max_index)
        for j, row in enumerate(rows):
            if j < csv_path[1]:
                continue
            if j > max_index -  csv_path[2]:
                break

            t = float(row['time'])
            command_x = float(row['command_linear_x'])
            command_y = float(row['command_linear_y'])
            command_z = float(row['command_angular_z'])
            # print(j, t, command_x, command_y, command_z)
            linear_x = float(row['linear_x'])
            linear_y = float(row['linear_y'])
            linear_z = float(row['linear_z'])
            base_x = float(row['base_x'])
            base_y = float(row['base_y'])
            base_z = float(row['base_z'])
            # cot = float(row['cot'])

            joint_vel = []
            joint_tor = []
            for joint_id in range(12):
                joint_vel.append(float(row['joint_velocity{}'.format(joint_id)]))
                joint_tor.append(float(row['joint_torque{}'.format(joint_id)]))

            joint_vel = np.array(joint_vel)
            joint_tor = np.array(joint_tor)

            timestamps.append(t)

            hor_vel = np.array([linear_x, linear_y])
            hor_speed = np.linalg.norm(hor_vel)

            # joint_positive_mech_power = np.maximum(np.multiply(joint_vel, joint_tor), 0.0)
            # cot_computed = np.sum(joint_positive_mech_power) / (np.maximum(0.0, hor_speed) * 9.81)

            vel_command_ = np.array([command_x, command_y])
            command_norm_ = np.linalg.norm(vel_command_)
            stand_command_ = np.array([0.0, 0.0, 0.0])

            if np.array_equal([command_x, command_y, command_z], stand_command_):
                continue


            if hor_speed > 0.1 and command_norm_ > 0.1 and abs(command_x) < 2.0:  # If commanded higher than 2.0, it is not my controller
                moving_speeds.append(hor_speed)
                base_positions.append((base_x, base_y))
                # if cot_computed > 100:
                #     print("debug ", joint_tor, "/",  vel_command_ , "/ ", hor_speed, "/ ", joint_positive_mech_power, "/ ", joint_tor,  "/ ", joint_vel)
                # cots_recorded.append(cot)

                joint_velocities.append(joint_vel)
                joint_torques.append(joint_tor)

    # Compute COT with filtered joint torque / speed
    def butter_lowpass(cutoff, fs, order=5):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    def butter_lowpass_filter(data, cutoff, fs, order=5):
        b, a = butter_lowpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y

    joint_velocities = np.stack(joint_velocities, axis=0)
    joint_torques = np.stack(joint_torques, axis=0)

    print(joint_velocities.shape)

    cots = []
    for data_idx in range(joint_velocities.shape[0]):
        joint_positive_mech_power = np.maximum(np.multiply(joint_velocities[data_idx], joint_torques[data_idx]), 0.0)
        cot_computed = np.sum(joint_positive_mech_power) / (np.maximum(0.0, moving_speeds[data_idx]) * 9.81)
        cot_computed /= robot_mass
        print(joint_velocities[data_idx], ", ", joint_torques[data_idx], ", ",joint_positive_mech_power)
        cots.append(cot_computed)

    time = np.arange(len(cots))

    fs = 20.0
    cutoff = 5.0
    order = 5
    # cot_filtered = butter_lowpass_filter(cots, cutoff, fs, order)
    cot_filtered = cots

    # Plot COT values
    plt.figure()
    plt.plot(cots, label="computed")
    # plt.plot(cots_recorded, label="recorded")
    # plt.plot(cot_filtered, label="filtered")
    #
    #
    # plt.legend()
    # # Show the plot
    plt.show()

    cots = cot_filtered

    total_distance = 0.0
    prev_pos = None
    for pos in base_positions:
        if prev_pos is not None:
            dx = pos[0] - prev_pos[0]
            dy = pos[1] - prev_pos[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)
            total_distance += distance
        prev_pos = pos

    print("Total distance traveled: {:.2f} meters".format(total_distance))
    print("Average moving speed: {:.2f} m/s".format(np.mean(moving_speeds)))
    print("Max moving speed: {:.2f} m/s".format(max(moving_speeds)))
    print("Average COT: {:.2f}".format(np.mean(cots)))

    print("max_cot ",  max(cots))

    all_time_stamps.append(timestamps)
    all_moving_speeds.append(moving_speeds)
    all_base_positions.append(base_positions)
    all_cot.append(cots)


print("Average COT: {:.2f}".format(np.mean( [num for sublist in all_cot for num in sublist])))
#
# # Histograms
# all_moving_speeds_flat = [num for sublist in all_moving_speeds for num in sublist]
# all_cot_flat = [num for sublist in all_cot for num in sublist]
#
# # Calculate and overlay mean value
# mean_value = np.mean(all_moving_speeds_flat)
# mean_value_cot = np.mean(all_cot_flat)
#
# median_value = np.median(all_moving_speeds_flat)
# median_value_cot = np.median(all_cot_flat)
#
# # Create subplots
# fig, axes = plt.subplots(2, 1, figsize=(2.55, 2.5))  # Set the figure size in inches (65 mm = 2.55 inches, 45 mm = 2.0 inches)
#
# num_bins = 8
#
# # Histogram of moving speeds
# n, bins, patches = axes[0].hist(all_moving_speeds_flat, bins=num_bins, color=color_list[1], alpha=0.7, density=True)
# axes[0].hist(all_moving_speeds_flat, bins=num_bins, color=color_list[1], alpha=0.7, density=True)
# axes[0].set_xticks([0.5, 1.0, 1.5, 2.0])
# axes[0].set_xlabel('Speed (m/s)', fontsize=7)
# # axes[0].set_ylabel('Density', fontsize=7)
# axes[0].tick_params(axis='both', labelsize=7)
# axes[0].axvline(median_value, color=color_list[0], linewidth=1)  # Add a vertical line at the mean value
#
# # Add value annotations above each bar in the first histogram
# for count, patch in zip(n, patches):
#     height = patch.get_height()
#     if count > 0:
#         axes[0].text(patch.get_x() + patch.get_width() / 2, height, f'{count:.2f}', ha='center', va='bottom', fontsize=6)
#
# axes[0].set_ylim(top=max(n) * 1.3)
#
# # Histogram of COT values
# n, bins, patches = axes[1].hist(all_cot_flat, bins=num_bins, color=color_list[1], alpha=0.7, density=True)
# axes[1].set_xlabel('COT', fontsize=7)
# # axes[1].set_ylabel('Density', fontsize=7)
# axes[1].tick_params(axis='both', labelsize=7)
# axes[1].axvline(median_value_cot, color=color_list[0], linewidth=1)  # Add a vertical line at the mean value
#
# # Add value annotations above each bar in the first histogram
# for count, patch in zip(n, patches):
#     height = patch.get_height()
#     if count > 0:
#         axes[1].text(patch.get_x() + patch.get_width() / 2, height, f'{count:.2f}', ha='center', va='bottom', fontsize=6)
#
# axes[1].set_ylim(top=max(n) * 1.3)
#
#
# # Adjust layout and minimize whitespace
# plt.tight_layout(pad=0.5)
# plt.subplots_adjust(bottom=0.2)
#
# # Save the figure
# plt.savefig('saved_images/histograms.svg')
# plt.show()

# Histograms
all_moving_speeds_flat = [num for sublist in all_moving_speeds for num in sublist]
all_cot_flat = [num for sublist in all_cot for num in sublist]

# Calculate and overlay mean value
mean_value = np.mean(all_moving_speeds_flat)
mean_value_cot = np.mean(all_cot_flat)

median_value = np.median(all_moving_speeds_flat)
median_value_cot = np.median(all_cot_flat)

print("mean_value ", mean_value)
print("median_value ", median_value)
print("mean_value_cot ", mean_value_cot)
print("median_value_cot ", median_value_cot)

anymal_mean_speed = 0.74
anymal_mean_cot = 0.5

# Create subplots
fig, axes = plt.subplots(1, 2, figsize=(2.8, 1.3))  # Set the figure size and create a 1x2 grid of subplots

num_bins = 20

# Histogram of moving speeds
n, bins, patches = axes[0].hist(all_moving_speeds_flat, bins=num_bins, color=color_list[0], alpha=0.7, density=True)
axes[0].set_xlabel('Speed (m/s)', fontsize=8)
axes[0].tick_params(axis='x', labelsize=8)
axes[0].axvline(mean_value, color=color_list[2], linewidth=2)  # Add a vertical line at the median value
axes[0].set_yticks([])

# Histogram of COT values
n, bins, patches = axes[1].hist(all_cot_flat, bins=num_bins, color=color_list[0], alpha=0.7, density=True)
axes[1].set_xlabel('Mech. COT', fontsize=8)
axes[1].tick_params(axis='x', labelsize=8)
axes[1].axvline(mean_value_cot, color=color_list[2], linewidth=2)  # Add a vertical line at the median value

axes[1].set_yticks([])

# Adjust layout and minimize whitespace
plt.tight_layout(pad=0.5)


# Save the figure
plt.subplots_adjust(wspace=0.05)
plt.show()

# # Draw paths
# fig, axes = plt.subplots(2, len(csv_paths), figsize=(6 * len(csv_paths), 6))
#
#
#
# for i, csv_path in enumerate(csv_paths):
#     # hist_ax = axes[1][i]
#     scatter_ax = axes[0][i]
#     path_ax = axes[1][i]
#     # hist_ax.hist(moving_speeds, bins=20, color='b', alpha=0.7)
#     # hist_ax.set_xlabel('Speed (m/s)')
#     # hist_ax.set_ylabel('Frequency')
#     # hist_ax.set_title('Histogram of Moving Speeds')
#     # hist_ax.grid(True)
#
#     moving_positions = [pos for pos, speed in zip(all_base_positions[i], all_moving_speeds[i])]
#     scatter = scatter_ax.scatter(*zip(*moving_positions),
#                                  # c=[speed for speed in all_moving_speeds[i]],
#                                  c=[cot for cot in all_cot[i]],
#                                  # cmap='plasma',
#                                  vmin=0.0,
#                                  vmax=0.5
#                                  )
#     scatter_ax.set_xlabel('x')
#     scatter_ax.set_ylabel('y')
#     scatter_ax.set_title('Base positions when moving')
#     scatter_ax.axis('equal')
#     fig.colorbar(scatter, ax=scatter_ax, label='Speed (m/s)',)
#
#     base_positions = all_base_positions[i]
#     path_ax.plot(*zip(*base_positions), label=f'Mission {i+1}')
#     path_ax.set_xlabel('x')
#     path_ax.set_ylabel('y')
#     path_ax.set_title('Position Trajectory')
#     path_ax.axis('equal')
#     path_ax.legend()
#
# # plt.colorbar(label='Speed magnitude (m/s)')
#
# plt.tight_layout()
# plt.savefig('path_vis.svg')
# plt.show()
