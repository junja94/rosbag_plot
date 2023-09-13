import matplotlib.pyplot as plt
import pandas as pd
import csv
import math
import numpy as np

def collect_dts(csv_paths):
    dt_list = []
    for path in csv_paths:
        path = path + '/planner_dts.csv'
        load_planner_dt = pd.read_csv(path)
        dt_df = load_planner_dt['dts'].dropna()
        percentile = dt_df.quantile(0.75)  # top 25%
        top = dt_df[dt_df > percentile]
        dt_list.extend(top.to_list())
    return dt_list

def compute_total_path_length(csv_paths):
    total_distances = []
    for path in csv_paths:
        path = path + '/motion_400hz2.csv'

        base_positions = []
        # Open the CSV file
        with open(path, 'r') as file:
            reader = csv.DictReader(file)

            rows = list(reader)  # Convert reader object to a list of rows
            max_index = len(rows) - 1  #
            for j, row in enumerate(rows):
                # Convert the linear positions to floats
                base_x = float(row['base_x'])
                base_y = float(row['base_y'])
                base_z = float(row['base_z'])
                base_positions.append((base_x, base_y, base_z))

            total_distance = 0.0
            prev_pos = None
            for pos in base_positions:
                if prev_pos is not None:
                    dx = pos[0] - prev_pos[0]
                    dy = pos[1] - prev_pos[1]
                    distance = math.sqrt(dx ** 2 + dy ** 2)
                    total_distance += distance
                prev_pos = pos
            total_distances.append(total_distance)
    return total_distances

def compute_tracking_error(csv_paths):
    total_distances = []
    for path in csv_paths:
        path = path + '/motion_20hz2.csv'

        errors = []
        # Open the CSV file
        with open(path, 'r') as file:
            reader = csv.DictReader(file)

            rows = list(reader)  # Convert reader object to a list of rows
            max_index = len(rows) - 1  #
            for j, row in enumerate(rows):
                # Convert the linear positions to floats

                command_x = float(row['command_linear_x'])
                command_y = float(row['command_linear_y'])
                command = np.array([command_x, command_y])

                linvel_x = float(row['linear_x'])
                linvel_y = float(row['linear_y'])
                linvel = np.array([linvel_x, linvel_y])

                command_norm = np.linalg.norm(command)

                if command_norm > 0.5:
                    tracking_error = np.linalg.norm(command - linvel)
                    if tracking_error > 0.5:
                        print (tracking_error)
                        print(command)
                        print(linvel)
                    errors.append(tracking_error)

                    if tracking_error > 1.0:
                        print(linvel)
                        print(command)

    return errors

art_narrow_paths = [
    'data/art_narrow6',
    'data/art_narrow8',
    'data/art_narrow9',
]
art_wide_paths = [
    'data/art_wide5',
    'data/art_wide6',
    'data/art_wide7',
    'data/art_wide8',
]

ours_paths = [
    'data/ours1', # different policy
    # 'data/ours5',
    # 'data/ours6',
    # 'data/ours8',
    # 'data/ours9',
    # 'data/ours10',
]

ours_tracking = [
    'data/ours_tracking',
    'data/ours_tracking2',
]
art_tracking = [
'data/art_tracking1',
'data/art_tracking2',
'data/art_tracking3',
]

art_narrow_dt_list = collect_dts(art_narrow_paths)
art_wide_dt_list = collect_dts(art_wide_paths)

art_narrow_distances = compute_total_path_length(art_narrow_paths)
art_wide_distances = compute_total_path_length(art_wide_paths)
ours_distances = compute_total_path_length(ours_paths)

ours_tracking_errors = compute_tracking_error(ours_tracking)
art_tracking_errors = compute_tracking_error(art_tracking)

ours_computation_times = [228, 269, 267, 564, 225, 233, 220, 214, 720, 243, 285, 334, 214, 762]
for i in range(len(ours_computation_times)):
    ours_computation_times[i] = float(ours_computation_times[i]) / 1000000.0
print(ours_computation_times)


print("Mean")
print(np.mean(ours_computation_times))
print(np.mean(art_wide_dt_list))

computation_times = [ours_computation_times,
                     # art_wide_dt_list,
                     art_narrow_dt_list]

path_lengths = [ours_distances,
                # art_wide_distances,
                art_narrow_distances]

print(ours_tracking)
tracking_errors = [ours_tracking_errors,
                   art_tracking_errors]

color_list = ["#648fff", "#dc267f"]
fig, axes = plt.subplots(nrows=1, ncols=2, figsize=(2.5, 1.5))
# plt.rcParams.update({'font.size': 12})

computation_box = axes[0].boxplot(computation_times, sym='+',
                                 widths=0.5, patch_artist=True)
for patch, color in zip(computation_box['boxes'], color_list[:3]):
    patch.set_facecolor(color)
axes[0].set_xticklabels([])  # Remove x-axis tick labels
axes[0].set_yscale('log')
# axes[0].set_ylabel('Time (s)')

axes[1].hist(tracking_errors[0], bins=10, color=color_list[0], alpha=0.5, density=True)
axes[1].hist(tracking_errors[1], bins=10, color=color_list[1], alpha=0.5, density=True)
# axes[1].set_ylabel('Frequency')
# axes[1].set_xlabel('Tracking Error')
axes[1].tick_params(axis='x', labelsize=7)
axes[1].tick_params(axis='y', labelsize=7)

fig.tight_layout()


fig, axes = plt.subplots(nrows=2, ncols=1, figsize=(1.3, 1.25), sharey=True, sharex=True)

num_bins = 24

speed_bins = np.linspace(0.0, 1.2, num_bins + 1)

print(np.mean(tracking_errors[0]))
print(np.mean(tracking_errors[1]))

axes[0].hist(tracking_errors[0], bins=speed_bins, color=color_list[0], density=True)
axes[1].hist(tracking_errors[1], bins=speed_bins, color=color_list[1], density=True)
for i in range(2):
    axes[i].tick_params(axis='x', labelsize=7)
    axes[i].tick_params(axis='y', labelsize=7)
    axes[i].set_xlim([0, 1.2])
    axes[i].set_yticks([])
fig.tight_layout()

plt.savefig('saved_images/tracking_hist.svg')


plt.show()