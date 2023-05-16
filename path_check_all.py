import csv
import matplotlib.pyplot as plt
import math
import numpy as np
import glob
from scipy.signal import butter, lfilter, freqz

csv_paths = [
    "data/0421_mission2/motion.csv",  # 0.863 km
    "data/0421_mission3/motion.csv",  # 0.703 km
    "data/0427_mission1/motion.csv",  # 1.811 km
    "data/0427_mission2/motion.csv",  # 1.432 km
    "data/0504_mission/motion.csv"    # 1.993 km
]

robot_mass = 63.8975

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

    with open(csv_path, mode='r') as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            t = float(row['time'])
            command_x = float(row['command_linear_x'])
            command_y = float(row['command_linear_y'])
            command_z = float(row['command_angular_z'])
            linear_x = float(row['linear_x'])
            linear_y = float(row['linear_y'])
            linear_z = float(row['linear_z'])
            base_x = float(row['base_x'])
            base_y = float(row['base_y'])
            base_z = float(row['base_z'])
            cot = float(row['cot'])

            joint_vel = []
            joint_tor = []
            for joint_id in range(16):
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
            stand_command_ = np.array([0.5, 0.5, 0.5])

            if np.array_equal([command_x, command_y, command_z], stand_command_):
                continue


            if hor_speed > 0.1 and command_norm_ > 0.2 and abs(command_x) < 2.0:  # If commanded higher than 2.0, it is not my controller
                moving_speeds.append(hor_speed)
                base_positions.append((base_x, base_y))
                # if cot_computed > 100:
                #     print("debug ", joint_tor, "/",  vel_command_ , "/ ", hor_speed, "/ ", joint_positive_mech_power, "/ ", joint_tor,  "/ ", joint_vel)
                cots_recorded.append(cot)

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


    cots = []
    for data_idx in range(joint_velocities.shape[0]):
        joint_positive_mech_power = np.maximum(np.multiply(joint_velocities[data_idx], joint_torques[data_idx]), 0.0)
        cot_computed = np.sum(joint_positive_mech_power) / (np.maximum(0.0, moving_speeds[data_idx]) * 9.81)
        cot_computed /= robot_mass
        cots.append(cot_computed)

    time = np.arange(len(cots))

    fs = 20.0
    cutoff = 2.0
    order = 6
    cot_filtered = butter_lowpass_filter(cots, cutoff, fs, order)

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

# Histogram of moving speeds
plt.figure()
all_moving_speeds_flat = [num for sublist in all_moving_speeds for num in sublist]
plt.hist(all_moving_speeds_flat, bins=10, color='b', alpha=0.7)
plt.xlabel('Speed (m/s)')
plt.ylabel('Frequency')
plt.title('Histogram of Moving Speeds')
plt.show()

# Draw paths
fig, axes = plt.subplots(2, len(csv_paths), figsize=(6 * len(csv_paths), 6))



for i, csv_path in enumerate(csv_paths):
    # hist_ax = axes[1][i]
    scatter_ax = axes[0][i]
    path_ax = axes[1][i]
    # hist_ax.hist(moving_speeds, bins=20, color='b', alpha=0.7)
    # hist_ax.set_xlabel('Speed (m/s)')
    # hist_ax.set_ylabel('Frequency')
    # hist_ax.set_title('Histogram of Moving Speeds')
    # hist_ax.grid(True)

    moving_positions = [pos for pos, speed in zip(all_base_positions[i], all_moving_speeds[i])]
    scatter = scatter_ax.scatter(*zip(*moving_positions),
                                 # c=[speed for speed in all_moving_speeds[i]],
                                 c=[cot for cot in all_cot[i]],
                                 # cmap='plasma',
                                 vmin=0.0,
                                 vmax=0.5
                                 )
    scatter_ax.set_xlabel('x')
    scatter_ax.set_ylabel('y')
    scatter_ax.set_title('Base positions when moving')
    scatter_ax.axis('equal')
    fig.colorbar(scatter, ax=scatter_ax, label='Speed (m/s)',)

    base_positions = all_base_positions[i]
    path_ax.plot(*zip(*base_positions), label=f'Mission {i+1}')
    path_ax.set_xlabel('x')
    path_ax.set_ylabel('y')
    path_ax.set_title('Position Trajectory')
    path_ax.axis('equal')
    path_ax.legend()

# plt.colorbar(label='Speed magnitude (m/s)')

plt.tight_layout()
plt.savefig('path_vis.svg')
plt.show()
