import csv
import matplotlib.pyplot as plt
import math
import numpy as np

csv_paths = [
    ("data/0421_mission2/motion.csv", 0, 0),
    ("data/0421_mission3/motion.csv", 0, 0),
    ("data/0427_mission1/motion.csv", 0, 0),
    ("data/0427_mission2/motion.csv", 0, 180),
    ("data/0504_mission/motion.csv", 0, 620),
    ("data/0522_mission/motion.csv", 0, 0),
]

robot_mass = 63.8975

all_base_positions = []

for i, csv_path in enumerate(csv_paths):
    base_positions = []

    with open(csv_path[0], mode='r') as csv_file:
        reader = csv.DictReader(csv_file)
        rows = list(reader)  # Convert reader object to a list of rows
        max_index = len(rows) - 1
        for j, row in enumerate(rows):
            if j < csv_path[1]:
                continue
            if j > max_index - csv_path[2]:
                break
            base_x = float(row['base_x'])
            base_y = float(row['base_y'])
            base_positions.append((base_x, base_y))

    # Space the base_positions every 0.5 meters
    spaced_base_positions = []
    prev_position = None
    for position in base_positions:
        if prev_position is None or math.dist(position, prev_position) >= 1.0:
            spaced_base_positions.append(position)
            prev_position = position

    all_base_positions.append(spaced_base_positions)


# Interpolate between the base positions to make it denser
interpolated_base_positions = []

for positions in all_base_positions:
    x_values = [position[0] for position in positions]
    y_values = [position[1] for position in positions]
    t = np.arange(len(positions))

    t_interp = np.linspace(0, len(positions) - 1, 10 * len(positions))  # Adjust the density of interpolation
    x_interp = np.interp(t_interp, t, x_values)
    y_interp = np.interp(t_interp, t, y_values)

    interpolated_positions = list(zip(x_interp, y_interp))
    interpolated_base_positions.append(interpolated_positions)


# Create a dictionary to count the occurrence of each position
position_counts = {}

# Count positions with a 3.0-meter margin
count_margin = 3.0

for positions in interpolated_base_positions:
    for position in positions:
        is_counted = False
        for counted_position, count in position_counts.items():
            if math.dist(position, counted_position) <= count_margin:
                position_counts[counted_position] += 1
                is_counted = True
                break
        if not is_counted:
            position_counts[position] = 1

# Merge positions that are closeby based on a 1.0-meter margin
merged_positions = []
merge_margin = 1.0

for position, count in position_counts.items():
    is_merged = False
    for merged in merged_positions:
        if math.dist(position, merged[0]) <= merge_margin:
            merged[0] = (
                (position[0] * count + merged[0][0] * merged[1]) / (count + merged[1]),
                (position[1] * count + merged[0][1] * merged[1]) / (count + merged[1]),
            )
            merged[1] += count
            is_merged = True
            break
    if not is_merged:
        merged_positions.append([[position[0], position[1]], count])


# Create a colormap based on the count of each position
counts = [count for _, count in merged_positions]
print(max(counts))
max_count_clip = 250

# Clip counts to max_count_clip
for i, count in enumerate(counts):
    if count > max_count_clip:
        counts[i] = max_count_clip
    counts[i] /= max_count_clip

x = [position[0][0] for position in merged_positions]
y = [position[0][1] for position in merged_positions]

# Scatter plot of all visited positions with colormap
plt.scatter(x, y, s=10, c=counts, cmap='hot')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Scatter Plot of Visited Positions')
plt.colorbar(label='Frequency')

plt.savefig('saved_images/path_vis.svg')

plt.show()
