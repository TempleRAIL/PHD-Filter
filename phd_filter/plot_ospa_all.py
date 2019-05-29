import matplotlib.pyplot as plt
import csv

x = []
y = []
y1 = []
y2 = []
y3 = []

with open('OSPA_cm1_average.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=' ')
    row1 = next(plots)
    for row in plots:
        x.append(float(row[0])-float(row1[0]))
        y.append(float(row[1]))
        y1.append(float(row[2]))
	y2.append(float(row[3]))
	y3.append(float(row[4]))

plt.rcParams.update({'font.size': 18})
plt.gca().set_color_cycle(['black', 'red', 'green', 'blue'])
#linestyles = ['-', '--', '-.', ':']

plt.plot(x,y, label='All Classes', linewidth=2)
plt.plot(x,y1, label='Person', linestyle=':', linewidth=1.5)
plt.plot(x,y2, label='Chair', linestyle=':', linewidth=1.5)
plt.plot(x,y3, label='Table', linestyle=':', linewidth=1.5)

plt.ylim(0, 10)
plt.xlabel('Time (s)', fontsize=18)
plt.ylabel('OSPA error (m)', fontsize=18)
plt.title('OSPA Errors')
plt.legend()
plt.show()

