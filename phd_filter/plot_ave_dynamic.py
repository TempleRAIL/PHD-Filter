import matplotlib.pyplot as plt
import csv

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

plt.rcParams.update({'font.size': 18})
data = [y, y1, y2]
plt.boxplot(data, widths=0.1, sym='k+', labels=['All classes', 'Person', 'Chair'], positions=[0.0,0.5,1.0])
plt.xlabel('Class')
plt.ylabel('OSPA error (m)')
plt.title('OSPA Errors')
plt.show()
