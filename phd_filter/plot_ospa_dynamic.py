import matplotlib.pyplot as plt
import csv
from scipy.signal import savgol_filter

x = []
y = []
y1 = []
y2 = []
y3 = []

with open('OSPA_dynamic.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=' ')
    row1 = next(plots)
    for row in plots:
        x.append(float(row[0])-float(row1[0]))
        y.append(float(row[1]))
        y1.append(float(row[2]))
	y2.append(float(row[3]))


plt.gca().set_color_cycle(['black', 'red', 'green', 'blue'])
#linestyles = ['-', '--', '-.', ':']

#y2 = savgol_filter(y1, 301, 1)
#plt.plot(x,y, label='All')
plt.plot(x,y1, label='Person', linewidth=1.0)
plt.plot(x,y2, label='Chair', linewidth=1.0)
#plt.plot(x,y2, label='Smooth', linestyle=':', color='red')
plt.ylim(0, 10)
plt.xlabel('Time (s)')
plt.ylabel('OSPA error (m)')
plt.title('OSPA Errors')
plt.legend()
plt.show()

