from math import *
def generate_table():
	lookup_table = []
	center = (10, 10)
	for row in range(21):
		for col in range(21):
			if row == center[0] and col == center[1]:
				continue
			delta_x = center[0] - row
			delta_y = col - center[1]
			if delta_x == 0 or delta_y == 0:
				continue
			else:
				degree = degrees(atan(delta_y/(delta_x)))
				if delta_y < 0 and delta_x > 0:
					degree += 360
				elif delta_x < 0:
					degree += 180
			node = [-delta_x, delta_y, degree]
			lookup_table.append(node)
	#add 8 same derections
	lookup_table.append([-10, 0,  0])
	lookup_table.append([-10, 10,  45])
	lookup_table.append([0, 10,  90])
	lookup_table.append([10, 10,  135])
	lookup_table.append([10, 0,  180])
	lookup_table.append([10, -10,  225])
	lookup_table.append([0, -10,  270])
	lookup_table.append([-10, -10,  315])
	#sorted by angle
	lookup_table.sort(key=lambda node:node[2])
	print('look up table generating...')
	return lookup_table

if __name__ == '__main__':
	table = generate_table()
	print('the look up table size is:', len(table))
	for i in range(len(table)):
		print('[%s %s %.2f]' % (table[i][0], table[i][1] , table[i][2]), end='')
		if i %10 == 0:
			print('\n')
