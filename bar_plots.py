import matplotlib.pyplot as plt

# Data (example values)
success_rate = [0.7, 0.2, 0.8, 0.5]  # Success rates for three controllers
collision_rate = [0.0, 0.0, 0.3, 1.0]  # Collision rates for three controllers


success_rate = [0.7, 0.2,  0.5]  # Success rates for three controllers
collision_rate = [0.0, 0.0, 1.0]  # Collision rates for three controllers

failure_rate = [1.0 - x for x in success_rate]


# controllers = ['Controller A', 'Controller B', 'Controller C']  # Names of the controllers
controllers = [0,1,2]  # Names of the controllers

color_list = ["#648fff", "#785ef0", "#dc267f", "#fe6100", "#ffb000", "#000000", "#ffffff"]

color_list = ["#648fff", "#785ef0", "#dc267f"]

# Creating a figure with two subplots sharing y-axis
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(2, 1.2), sharey=True)  # Adjust the figure size as needed
plt.rcParams.update({'font.size': 7})  # Set the font size

# Plotting the success rate
ax1.bar(controllers, failure_rate, color=color_list)
ax1.set_xticklabels([])  # Remove x-axis tick labels
# ax1.set_title('Success Rate')
ax1.tick_params(axis='y', labelsize=7)

# ax1.set_ylabel('Rate')
# ax1.set_title('Comparison of Success Rate and Collision Rate for Three Controllers')

# Plotting the collision rate
ax2.bar(controllers, collision_rate, color=color_list)
ax2.set_xticklabels([])  # Remove x-axis tick labels
# ax2.set_title('Collision Rate')
ax2.tick_params(axis='y', labelsize=7)

# Setting the same y-axis limits for both subplots
min_rate = min(min(success_rate), min(collision_rate))
max_rate = max(max(success_rate), max(collision_rate))
ax1.set_ylim([min_rate, max_rate])
ax2.set_ylim([min_rate, max_rate])

plt.tight_layout()  # To prevent overlapping of labels and titles
plt.savefig('saved_images/bars.svg')

plt.show()
