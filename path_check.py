import csv
import matplotlib.pyplot as plt
import math
import glob

# csv_path = "data/0421_mission2/linear_velocities.csv"  # 0.863 km
# csv_path = "data/0421_mission3/linear_velocities.csv"  # 0.703 km
# csv_path = "data/0427_mission1/linear_velocities.csv"  # 1.811 km
# csv_path = "data/0427_mission2/linear_velocities.csv"  # 1.432 km
csv_path = "data/0504_mission/linear_velocities.csv"  # 1.993 km

# read linear velocities and corresponding base positions from CSV file
timestamps = []
linear_speeds = []
moving_speeds = []
base_positions = []
with open(csv_path, mode='r') as csv_file:
    reader = csv.DictReader(csv_file)
    for row in reader:
        t = float(row['time'])
        linear_x = float(row['linear_x'])
        linear_y = float(row['linear_y'])
        linear_z = float(row['linear_z'])
        base_x = float(row['base_x'])
        base_y = float(row['base_y'])
        base_z = float(row['base_z'])
        timestamps.append(t)
        hor_speed = math.sqrt(linear_x ** 2 + linear_y ** 2)
        linear_speeds.append(hor_speed)
        if hor_speed > 0.1:
            moving_speeds.append(hor_speed)

        base_positions.append((base_x, base_y))

# compute total distance
total_distance = 0.0
prev_pos = None
for pos in base_positions:
    if prev_pos is not None:
        dx = pos[0] - prev_pos[0]
        dy = pos[1] - prev_pos[1]
        distance = math.sqrt(dx ** 2 + dy ** 2)
        total_distance += distance
    prev_pos = pos


# compute average moving speed
num_speeds = 0
sum_speeds = 0.0
for speed in moving_speeds:
    sum_speeds += speed
    num_speeds += 1

avg_moving_speed = sum_speeds / num_speeds if num_speeds > 0 else 0.0

print("Total distance traveled: {:.2f} meters".format(total_distance))
print("Average moving speed: {:.2f} m/s".format(avg_moving_speed))
print("Max moving speed: {:.2f} m/s".format(max(linear_speeds)))


# plot histogram of moving speeds
plt.figure()
plt.hist(moving_speeds, bins=20, color='b', alpha=0.7)
plt.xlabel('Speed (m/s)')
plt.ylabel('Frequency')
plt.title('Histogram of Moving Speeds')
plt.grid(True)
plt.show()

# plot base positions when moving
moving_positions = [pos for pos, speed in zip(base_positions, linear_speeds) if speed > 0.1]
plt.figure()
plt.scatter(*zip(*moving_positions), c=[speed for speed in linear_speeds if speed > 0.1], cmap='bwr')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Base positions when moving')
plt.colorbar(label='Speed magnitude (m/s)')
plt.show()