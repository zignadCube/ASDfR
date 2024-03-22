import numpy as np
import matplotlib.pyplot as plt

num_bins = 500
fig, axs = plt.subplots(2)
first_range = 0.99e6
last_range = 2.0e6


with open("Assignment2/measurements_ros2_nodes_wo_stress.txt", 'r') as f:
    data = f.readlines()[0].split(',')[:-1]
    timings = [int(x) for x in data]
    print(f"Base wo stress: ({len(timings)} samples):")
    print("Mean: ", np.mean(timings))
    print("Median: ", np.median(timings))
    print("Standard Deviation: ", np.std(timings))
    print("Variance: ", np.var(timings))

    # Determine range of measurements
    min_timing = min(timings)
    max_timing = max(timings)
    range_timing = max_timing - min_timing
    print("Max: ", max_timing, "Min: ", min_timing, "Range: ", range_timing)
    print('\n')
    axs[0].hist(timings, bins=num_bins, range=(first_range, last_range))
    axs[0].grid()
    axs[0].set_title('Timing measurements without stress')
    

with open("Assignment2/measurements_ros2_nodes_w_stress.txt", 'r') as f:
    data = f.readlines()[0].split(',')[:-1]
    timings = [int(x) for x in data]
    print(f"Base w stree: ({len(timings)} samples):")
    print("Mean: ", np.mean(timings))
    print("Median: ", np.median(timings))
    print("Standard Deviation: ", np.std(timings))
    print("Variance: ", np.var(timings))

    # Determine range of measurements
    min_timing = min(timings)
    max_timing = max(timings)
    range_timing = max_timing - min_timing
    print("Max: ", max_timing, "Min: ", min_timing, "Range: ", range_timing)
    print('\n')
    plt.hist
    axs[1].hist(timings, bins=num_bins, range=(first_range, last_range))
    axs[1].grid()
    axs[1].set_title('Timing measurements with stress')

  

plt.show()
