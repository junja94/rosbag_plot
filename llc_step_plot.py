import matplotlib.pyplot as plt

# Define speed range and step height range
step_height_range = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]

# Dummy data for max step height at each speed
speed_range             = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]


# Going up the steps:
# 0.15 m: all speeds
# 0.2 m: 0.4 m/s and above
# 0.25 m: 0.4 m/s and above
# 0.3 m: 0.6 m/s and above
# 0.35 m: 0.8 m/s and above (with collision)
# 0.4 m: 1.2 m/s and above
# 0.45 m: not possible

# down
# 0.2 m: all speeds
# 0.3 m: 0.4 m/s and above
# 0.4 m: 0.6 m/s and above
# 0.45 m: 0.8 m/s and above
# 0.5 m: 1.2 m/s and above
# 0.55 m: 1.4 m/s and above
# 0.6 m: 1.8 m/s and above

max_step_height_data_up = [0.15, 0.25, 0.3, 0.35, 0.35, 0.4, 0.4, 0.4, 0.4, 0.4]
max_step_height_data_down = [0.2, 0.3, 0.4, 0.45, 0.45, 0.5, 0.55, 0.55, 0.6, 0.6]


color_list = ["#648fff", "#785ef0", "#dc267f"]

# Set the figure size
plt.figure(figsize=(2.0, 1.5))

# Create the plot
plt.plot(speed_range, max_step_height_data_up, marker='o', color=color_list[0])
plt.plot(speed_range, max_step_height_data_down, marker='s', color=color_list[2])

# Set the labels and title
# plt.xlabel('Speed (m/s)')
# plt.ylabel('Maximum Step Height (m)')
# plt.title('Max. Step Height vs. Speed')

# Set the x and y axis limits
plt.xlim(0.2, 2.0)
# plt.yticks(step_height_range)
plt.ylim(0.1, 0.65)

# plt.yticks(slope_range)

plt.tick_params(axis='x', labelsize=8)
plt.tick_params(axis='y', labelsize=8)

# Show the gridlines
# plt.grid(True)
plt.tight_layout()

# Display the plot
plt.savefig('saved_images/step_height.svg')
plt.show()


# Slope
# 10 deg: up and down works
# 20 deg: up and down works
# 25 deg: all
# 30 deg: from 0.6 m/s


slope_range = [0, 10, 20, 30, 40]
speed_range = [0.2, 0.4, 0.6, 0.8, 1.0]
max_slope  = [27, 27, 31, 31, 31]

plt.figure(figsize=(2.0, 1.25))

plt.plot(speed_range, max_slope, marker='s', color=color_list[2])

plt.ylim(26, 32)

plt.tick_params(axis='x', labelsize=8)
plt.tick_params(axis='y', labelsize=8)
plt.tight_layout()

# Display the plot
plt.savefig('saved_images/slope.svg')
plt.show()
