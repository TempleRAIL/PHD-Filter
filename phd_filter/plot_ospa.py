import matplotlib.pyplot as plt
import csv

x = []
y = []
y1 = []
y2 = []
y3 = []

with open('OSPA.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=' ')
    row1 = next(plots)
    for row in plots:
        x.append(float(row[0])-float(row1[0]))
        y.append(float(row[1]))
        y1.append(float(row[2]))
	y2.append(float(row[3]))
	y3.append(float(row[4]))

plt.gca().set_color_cycle(['red', 'green', 'blue', 'yellow'])
linestyles = ['-', '--', '-.', ':']

plt.plot(x,y, label='All Classes')
plt.plot(x,y1, label='Person', linestyle=':', linewidth=1)
plt.plot(x,y2, label='Chair', linestyle=':', linewidth=1)
plt.plot(x,y3, label='Table', linestyle=':', linewidth=1)

plt.xlabel('Time (s)')
plt.ylabel('OSPA error (m)')
plt.title('OSPA Errors')
plt.legend()
plt.show()

