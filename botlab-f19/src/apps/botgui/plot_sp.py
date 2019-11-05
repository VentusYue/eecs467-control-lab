import matplotlib.pyplot as plt
import csv
import numpy as np


time = []
left_wheel = []
# right_wheel = [] # produce plots one at a time

with open('critically.csv') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        time.append(int(row[0]) - int (1565118114557460))
        left_wheel.append(row[1])
        # right_wheel.append(int(row[2]))

plt.plot(time, left_wheel, label='Left wheel')
# plt.plot(time, right_wheel, label='Right wheel')
plt.xlabel('Time')
plt.ylabel('velocity')
plt.title('Left Overdamped')  # underdamped, well_tuned
plt.show()