import csv
import matplotlib.pyplot as plt
import math
import numpy as np
import glob
from scipy.signal import butter, lfilter, freqz


csv_paths = [
    ("data/0421_mission2/motion_400hz2.csv", 0, 0),
    ("data/0421_mission3/motion_400hz2.csv", 0, 0),
    # ("data/0427_mission1/motion_400hz2.csv", 0, 0),
    # ("data/0427_mission2/motion_400hz2.csv", 0, 720),
    # ("data/0504_mission/motion_400hz2.csv", 0, 2480),
    # ("data/0522_mission/motion_400hz2.csv", 0, 0),
    # ("data/anymal_caiman_darpa/motion_400hz2.csv", 0, 0),
]
num_joint = 16
robot_mass = 65

# num_joint = 12
# robot_mass = 52

# color_list = ["#fd7f6f", "#7eb0d5", "#b2e061", "#bd7ebe", "#ffb55a", "#ffee65", "#beb9db", "#fdcce5", "#8bd3c7"]

#IBM color blind safe pallette
color_list = ["#648fff", "#785ef0", "#dc267f", "#fe6100", "#ffb000", "#000000", "#ffffff"]




all_time_stamps = []
all_moving_speeds = []
all_cot = []
all_cot_walking = []
all_cot_driving = []

all_torques = []
all_torques_walking = []
all_torques_driving = []

all_wheel_torques_walking = []
all_wheel_torques_driving = []

all_leg_torques_walking = []
all_leg_torques_driving = []

all_wheel_cot_walking = []
all_wheel_cot_driving = []

all_leg_cot_walking = []
all_leg_cot_driving = []

for i, csv_path in enumerate(csv_paths):
    timestamps = []
    moving_speeds = []
    joint_velocities = []
    joint_torques = []

    wheel_velocities = []
    wheel_torques = []

    with open(csv_path[0], mode='r') as csv_file:
        reader = csv.DictReader(csv_file)
        rows = list(reader)  # Convert reader object to a list of rows
        max_index = len(rows) - 1  #
        print(max_index)
        for j, row in enumerate(rows):
            if j < csv_path[1]:
                continue
            if j > max_index - csv_path[2]:
                break

            t = float(row['time'])
            command_x = float(row['command_linear_x'])
            command_y = float(row['command_linear_y'])
            command_z = float(row['command_angular_z'])
            # print(j, t, command_x, command_y, command_z)
            linear_x = float(row['linear_x'])
            linear_y = float(row['linear_y'])
            linear_z = float(row['linear_z'])

            joint_vel = []
            joint_tor = []

            wheel_vel = []
            wheel_tor = []

            for joint_id in range(num_joint):

                if num_joint == 16:
                    if joint_id % 4 == 3:
                        wheel_vel.append(float(row['joint_velocity{}'.format(joint_id)]))
                        wheel_tor.append(float(row['joint_torque{}'.format(joint_id)]))
                    else:
                        joint_vel.append(float(row['joint_velocity{}'.format(joint_id)]))
                        joint_tor.append(float(row['joint_torque{}'.format(joint_id)]))
                else:
                    joint_vel.append(float(row['joint_velocity{}'.format(joint_id)]))
                    joint_tor.append(float(row['joint_torque{}'.format(joint_id)]))


            joint_vel = np.array(joint_vel)
            joint_tor = np.array(joint_tor)

            wheel_vel = np.array(wheel_vel)
            wheel_tor = np.array(wheel_tor)


            timestamps.append(t)

            hor_vel = np.array([linear_x, linear_y])
            hor_speed = np.linalg.norm(hor_vel)

            vel_command_ = np.array([command_x, command_y])
            command_norm_ = np.linalg.norm(vel_command_)
            stand_command_ = np.array([0.5, 0.5, 0.5])

            if np.array_equal([command_x, command_y, command_z], stand_command_):
                continue

            if hor_speed > 0.2 and command_norm_ > 0.5 and abs(command_x) < 2.0:  # If commanded higher than 2.0, it is not my controller
                moving_speeds.append(hor_speed)
                joint_velocities.append(joint_vel)
                joint_torques.append(joint_tor)
                wheel_velocities.append(wheel_vel)
                wheel_torques.append(wheel_tor)


    joint_velocities = np.stack(joint_velocities, axis=0)
    joint_torques = np.stack(joint_torques, axis=0)

    wheel_velocities = np.stack(wheel_velocities, axis=0)
    wheel_torques = np.stack(wheel_torques, axis=0)


    print(joint_velocities.shape)
    print(wheel_velocities.shape)

    print(np.mean(np.abs(joint_velocities)))
    print(np.max(np.abs(joint_velocities)))

    cots = []
    cots_walking = []
    cots_driving = []
    torques = []
    torques_walking = []
    torques_driving = []


    wheel_torques_walking = []
    wheel_torques_driving = []

    leg_torques_walking = []
    leg_torques_driving = []

    wheel_cot_driving = []
    wheel_cot_walking = []

    leg_cot_driving = []
    leg_cot_walking = []

    for data_idx in range(joint_velocities.shape[0]):
        mechanical_power_leg = np.sum(np.clip(joint_velocities[data_idx] * joint_torques[data_idx], 0.0, None))
        mechanical_power_wheel = np.sum(np.clip(wheel_velocities[data_idx] * wheel_torques[data_idx], 0.0, None))
        cot_computed_leg = (mechanical_power_leg) / (np.clip(moving_speeds[data_idx], 0.01, None) * 9.81)
        cot_computed_leg /= robot_mass
        cot_computed_wheel = (mechanical_power_wheel)  / (np.clip(moving_speeds[data_idx], 0.01, None) * 9.81)
        cot_computed_wheel /= robot_mass

        cot_computed = cot_computed_wheel + cot_computed_leg

        # joint_torque_squared_sum = np.sum(np.square(joint_torques[data_idx]))
        joint_torque_squared_sum = np.mean(np.abs(joint_torques[data_idx]))
        # joint_torque_squared_sum = np.mean(np.abs(joint_velocities[data_idx]))

        # joint_torque_squared_sum  /= (np.clip(moving_speeds[data_idx], 0.01, None)
        # joint_torque_squared_sum /= robot_mass

        # wheel_torque_squared_sum = np.sum(np.square(wheel_torques[data_idx]))
        wheel_torque_squared_sum = np.mean(np.abs(wheel_torques[data_idx]))
        # wheel_torque_squared_sum = np.mean(np.abs(wheel_velocities[data_idx]))
        # wheel_torque_squared_sum /= (np.clip(moving_speeds[data_idx], 0.01, None) * 9.81)
        # wheel_torque_squared_sum /= robot_mass

        if (np.max(np.abs(joint_velocities[data_idx]))) > 1.0:
            cots_walking.append(cot_computed)
            torques_walking.append(joint_torque_squared_sum)
            wheel_torques_walking.append(wheel_torque_squared_sum)
            leg_torques_walking.append(joint_torque_squared_sum)

            wheel_cot_driving.append(cot_computed_wheel)
            leg_cot_driving.append(cot_computed_leg)
        else:
            cots_driving.append(cot_computed)
            torques_driving.append(joint_torque_squared_sum)
            wheel_torques_driving.append(wheel_torque_squared_sum)
            leg_torques_driving.append(joint_torque_squared_sum)

            wheel_cot_walking.append(cot_computed_wheel)
            leg_cot_walking.append(cot_computed_leg)

        # joint_positive_mech_power = np.maximum(np.multiply(joint_velocities[data_idx], joint_torques[data_idx]), 0.0)
        # cot_computed = np.sum(joint_positive_mech_power) / (np.maximum(0.0, moving_speeds[data_idx]) * 9.81)
        cots.append(cot_computed)
        torques.append(joint_torque_squared_sum)


    time = np.arange(len(cots))

    fs = 400.0
    cutoff = 5.0
    order = 5

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

    # cot_filtered = butter_lowpass_filter(cots, cutoff, fs, order)
    cot_filtered = cots

    # Plot COT values
    # plt.figure()
    # plt.plot(cots, label="computed")
    # plt.plot(cots_recorded, label="recorded")
    # plt.plot(cot_filtered, label="filtered")
    #
    #
    # plt.legend()
    # # Show the plot
    # plt.show()
    cots = cot_filtered


    print("Average moving speed: {:.2f} m/s".format(np.mean(moving_speeds)))
    print("Max moving speed: {:.2f} m/s".format(max(moving_speeds)))
    print("Average COT: {:.2f}".format(np.mean(cots)))

    print("max_cot ",  max(cots))

    all_time_stamps.append(timestamps)
    all_moving_speeds.append(moving_speeds)
    all_cot.append(cots)
    all_cot_walking.append(cots_walking)
    all_cot_driving.append(cots_driving)

    all_torques.append(torques)
    all_torques_walking.append(torques_walking)
    all_torques_driving.append(torques_driving)

    all_wheel_torques_walking.append(wheel_torques_walking)
    all_wheel_torques_driving.append(wheel_torques_driving)

    all_leg_torques_walking.append(leg_torques_walking)
    all_leg_torques_driving.append(leg_torques_driving)

    all_wheel_cot_walking.append(wheel_cot_walking)
    all_wheel_cot_driving.append(wheel_cot_driving)

    all_leg_cot_walking.append(leg_cot_walking)
    all_leg_cot_driving.append(leg_cot_driving)


print("Average COT: {:.2f}".format(np.mean( [num for sublist in all_cot for num in sublist])))
#
# Histograms
all_moving_speeds_flat = [num for sublist in all_moving_speeds for num in sublist]
all_cot_flat = [num for sublist in all_cot for num in sublist]
all_cot_walking_flat = [num for sublist in all_cot_walking for num in sublist]
all_cot_driving_flat = [num for sublist in all_cot_driving for num in sublist]
all_torques_flat = [num for sublist in all_torques for num in sublist]
all_torques_walking_flat = [num for sublist in all_torques_walking for num in sublist]
all_torques_driving_flat = [num for sublist in all_torques_driving for num in sublist]

all_wheel_torques_walking_flat = [num for sublist in all_wheel_torques_walking for num in sublist]
all_wheel_torques_driving_flat = [num for sublist in all_wheel_torques_driving for num in sublist]

all_leg_torques_walking_flat = [num for sublist in all_leg_torques_walking for num in sublist]
all_leg_torques_driving_flat = [num for sublist in all_leg_torques_driving for num in sublist]

all_wheel_cot_walking_flat = [num for sublist in all_wheel_cot_walking for num in sublist]
all_wheel_cot_driving_flat = [num for sublist in all_wheel_cot_driving for num in sublist]

all_leg_cot_walking_flat = [num for sublist in all_leg_cot_walking for num in sublist]
all_leg_cot_driving_flat = [num for sublist in all_leg_cot_driving for num in sublist]



def get_mean_median(data):
    median = np.median(data)
    mean = np.mean(data)
    print(median, "/", mean)
    return mean, median

# Calculate and overlay mean value

mean_value_speed, median_value_speed = get_mean_median(all_moving_speeds_flat)
mean_value_cot, median_value_cot = get_mean_median(all_cot_flat)

mean_value_cot_walking, median_value_cot_walking = get_mean_median(all_cot_walking_flat)
mean_value_cot_driving, median_value_cot_driving = get_mean_median(all_cot_driving_flat)

mean_value_torques, median_value_torques = get_mean_median(all_torques_flat)

mean_value_torques_walking, median_value_torques_walking = get_mean_median(all_torques_walking_flat)
mean_value_torques_driving, median_value_torques_driving = get_mean_median(all_torques_driving_flat)

print("torque")
mean_value_wheel_torques_walking, median_value_wheel_torques_walking = get_mean_median(all_wheel_torques_walking_flat)
mean_value_wheel_torques_driving, median_value_wheel_torques_driving = get_mean_median(all_wheel_torques_driving_flat)

mean_value_leg_torques_walking, median_value_leg_torques_walking = get_mean_median(all_leg_torques_walking_flat)
mean_value_leg_torques_driving, median_value_leg_torques_driving = get_mean_median(all_leg_torques_driving_flat)

print("cot")

mean_value_wheel_cot_walking, median_value_wheel_cot_walking = get_mean_median(all_wheel_cot_walking_flat)
mean_value_wheel_cot_driving, median_value_wheel_cot_driving = get_mean_median(all_wheel_cot_driving_flat)

mean_value_leg_cot_walking, median_value_leg_cot_walking = get_mean_median(all_leg_cot_walking_flat)
mean_value_leg_cot_driving, median_value_leg_cot_driving = get_mean_median(all_leg_cot_driving_flat)




print("mean_value_speed ", mean_value_speed)
print("median_value_speed ", median_value_speed)
print("mean_value_cot ", mean_value_cot)
print("median_value_cot ", median_value_cot)

clip_cot = 1.2
for i in range (len(all_cot_flat)):
    if all_cot_flat[i] > clip_cot:
        all_cot_flat[i] = clip_cot

anymal_mean_speed = 0.55
anymal_mean_cot = 0.35

# Create subplots
fig, axes = plt.subplots(1, 2, figsize=(2.5, 1.3))  # Set the figure size and create a 1x2 grid of subplots

num_bins = 20

speed_bins = np.linspace(0.0, 2.5, num_bins + 1)
cot_bins = np.linspace(0.0, 0.75, num_bins + 1)

# Histogram of moving speeds
n, bins, patches = axes[0].hist(all_moving_speeds_flat, bins=speed_bins, color=color_list[0], alpha=0.7, density=True)
axes[0].set_xticks([0.5, 1.0, 1.5, 2.0])
axes[0].set_xlabel('Speed (m/s)', fontsize=8)
axes[0].tick_params(axis='x', labelsize=8)
axes[0].axvline(mean_value_speed, color=color_list[2], linewidth=2)  # Add a vertical line at the median value
# axes[0].axvline(median_value, color=color_list[0], linewidth=2)  # Add a vertical line at the median value
axes[0].axvline(anymal_mean_speed, color=color_list[1], linewidth=2)  # Add a vertical line at the median value

# # Add value annotations above each bar in the first histogram
# for count, patch in zip(n, patches):
#     height = patch.get_height()
#     if count > 0:
#         val = count * patch.get_width()
#         axes[0].text(patch.get_x() + patch.get_width() / 2, height, f'{val:.2f}', ha='center', va='bottom', rotation='vertical', fontsize=6)
print(max(n))
axes[0].set_ylim(top=2.7 * 1.3)
axes[0].set_yticks([])
axes[0].set_xlim(right=2.5)

# Histogram of COT values
n, bins, patches = axes[1].hist(all_cot_flat, bins=cot_bins, color=color_list[0], alpha=0.7, density=True)
axes[1].set_xlabel('Mech. COT', fontsize=8)
axes[1].tick_params(axis='x', labelsize=8)
axes[1].axvline(mean_value_cot, color=color_list[2], linewidth=2)  # Add a vertical line at the median value
# axes[1].axvline(median_value_cot, color=color_list[0], linewidth=2)  # Add a vertical line at the median value
axes[1].axvline(anymal_mean_cot, color=color_list[1], linewidth=2)  # Add a vertical line at the median value
# axes[1].set_xlim(right=1.0)
axes[1].set_xlim(right=0.75)
axes[1].set_ylim(top=7.0 * 1.3)

# Add value annotations above each bar in the second histogram
# for count, patch in zip(n, patches):
#     height = patch.get_height()
#     if count > 0:
#         val = count * patch.get_width()
#         axes[1].text(patch.get_x() + patch.get_width() / 2, height, f'{val:.2f}', ha='center', va='bottom', rotation='vertical', fontsize=6)

# axes[1].set_ylim(top=max(n) * 1.3)
axes[1].set_yticks([])

# Adjust layout and minimize whitespace
plt.tight_layout(pad=0.5)

# Save the figure
plt.subplots_adjust(wspace=0.05)
plt.savefig('saved_images/histograms.svg')
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
