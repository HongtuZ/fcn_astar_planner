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

def generate_circle_table():
	lookup_table = []
	#0-45 degrees
	primitive = [(-10,0,0),(-10,1,5.7),(-10,2,11.3),(-10,3,16.7),(-9,3,18.4),(-9,4,24),(-9,5,29.1),(-8,5,32),(-8,6,36.9),(-7,7,45)]
	lookup_table += primitive
	#45-90 degrees
	for p in reversed(primitive):
		n = (-p[1],-p[0],90-p[2])
		if n not in lookup_table:
			lookup_table.append(n)
	#90-180	
	for p in reversed(lookup_table):
		n = (-p[0],p[1],180-p[2])
		if n not in lookup_table:
			lookup_table.append(n)
	#180-360	
	for p in reversed(lookup_table):
		n = (p[0],-p[1],360-p[2])
		if n not in lookup_table:
			lookup_table.append(n)
	lookup_table.pop()
	return lookup_table

if __name__ == '__main__':
	table = generate_circle_table()
	'''
	for i in range(len(table)):
		print('[%s %s %.2f]' % (table[i][0], table[i][1] , table[i][2]), end='')
		if i %10 == 9:
			print('\n')
	'''
	print(table)
	print('lookup_table size:', len(table))
	d = {}
	for i, p in enumerate(table):
		d[p[2]] = i
	print(len(d))
