import numpy as np

import matplotlib.pyplot as plt

color_list = ["#648fff", "#dc267f", "#fe6100"]

def draw_patterns(ax, pattern, dt):
    data_cnt = np.shape(pattern)[0]
    gap = 0.06

    for j in range(4):
        print("idx_", j)
        start_pos = 0
        n_timesteps = 0
        for data_idx in range(data_cnt):
            print(pattern[data_idx, j])
            if pattern[data_idx, j] == 1:
                if n_timesteps == 0:
                    start_pos = dt * data_idx
                n_timesteps += 1

            if (pattern[data_idx, j] == 0 and n_timesteps != 0) or data_idx == data_cnt - 1:

                print((pattern[data_idx, j] == 0 and n_timesteps != 0) )
                print("add plot", start_pos, ", ", dt * n_timesteps)
                ax.barh(y=gap * (3-j), width= dt * n_timesteps,
                                            height=0.05,
                                            align='center',
                                            color=color_list[0],
                                            left=start_pos,
                                            linewidth=0.01)
                start_pos = 0.0
                n_timesteps = 0


def generate_sequence(max_time, timestamps, dt):

    time_array = np.zeros(int(max_time / dt))
    print(time_array)

    start_times = []
    end_times = []
    for i, timestamp in enumerate(timestamps):
        if i % 2 == 0:
            print(timestamp)
            print(dt)
            print(timestamp/dt)
            start_times.append(round(timestamp / dt))
        else:
            end_times.append(round(timestamp / dt))

        if i == len(timestamps) - 1:
            if len(start_times)>len(end_times):
                end_times.append(round(max_time / dt))

    for i, timestamp in enumerate(start_times):
        time_array[timestamp:end_times[i]] = 1

    return time_array

max_time = 2.

# # PP0
timestamps =[
    np.array([0, 2, 7, 12, 17, 22, 27, 31, 37, 41]) * 0.05,
    np.array([0, 0, 2, 7, 12, 17, 22, 27, 32, 37, 41, 47, 52, 58])*0.05,
    np.array([0,0, 3, 7, 12, 18, 22, 27, 33, 36, 47,]) * 0.05,
    np.array([0, 2, 7, 11, 16, 22, 26, 32, 36, 41, 46, 51]) * 0.05,
    ]

# P30
# timestamps =[
#     np.array([0, 1, 10, 15, 22, 28, 34, 40, 47]) * 0.05,
#     np.array([0,0, 2, 11, 15, 24, 28, 35])*0.05,
#     np.array([0,0, 3, 9, 16, 22, 29, 34, 40]) * 0.05,
#     np.array([0,2, 8, 15, 22, 28, 35, 41 ]) * 0.05,
#     ]

dt = 0.05
contact_pattern = np.zeros((int(max_time / dt), 4), dtype=int)

for i, leg_timestamps in enumerate(timestamps):
    contact_pattern[:, i] = generate_sequence(max_time, leg_timestamps, dt)

fig, ax = plt.subplots(figsize=(2.1, 0.8))
draw_patterns(ax, contact_pattern, dt)
ax.set_xticks([0.0, 1.0, 2.0, 3.0])
ax.set_yticks([])
ax.tick_params(axis='both', labelsize=8)
# ax.set_title('Gait Pattern', fontsize=8)

# ax.set_xlim([0.0, 3.0])
ax.set_xlim([0.0, max_time])

plt.tight_layout()  # To prevent overlapping of labels and titles
plt.savefig('saved_images/gait_bars.svg')

plt.show()
