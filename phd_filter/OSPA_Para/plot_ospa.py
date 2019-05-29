import matplotlib.pyplot as plt
import csv

x1_ = []
x2_ = []
x3_ = []
y1_ = []
y2_ = []
y3_ = []

with open('OSPA_person_cm3_average.txt','r') as person:
	plots1 = csv.reader(person, delimiter=' ')
	for row in plots1:
		x1_.append(float(row[0]))
		y1_.append(float(row[1]))

with open('OSPA_chair_cm3_average.txt','r') as chair:
	plots2 = csv.reader(chair, delimiter=' ')
	for row in plots2:
		x2_.append(float(row[0]))
		y2_.append(float(row[1]))

with open('OSPA_table_cm3_average.txt','r') as table:
	plots3 = csv.reader(table, delimiter=' ')
	for row in plots3:
		x3_.append(float(row[0]))
		y3_.append(float(row[1]))		

plt.rcParams.update({'font.size': 18})
plt.gca().set_color_cycle(['red', 'green', 'blue'])
plt.plot(x1_,y1_, label='Person', linestyle=':', linewidth=1.5)
plt.plot(x2_,y2_, label='Chair', linestyle=':', linewidth=1.5)
plt.plot(x3_,y3_, label='Table', linestyle=':', linewidth=1.5)

plt.ylim(0, 10)
plt.xlabel('Time (s)', fontsize=18)
plt.ylabel('OSPA error (m)', fontsize=18)
plt.title('OSPA Errors')
plt.legend()
plt.show()


