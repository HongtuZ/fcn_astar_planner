from heapq import *
from car import Car
import tiev_constant as tc
from lookup_table import *
from math import *
from datetime import datetime
import random
import numpy as np
import cv2

VISUALIZE = False

class Node(object):
	
	def __init__(self, x=-1, y=-1, ang=0):
		self.x = x
		self.y = y
		self.ang = ang
		self.g = 0
		self.h = 10**8
		self.pre_node = None
	
	def __lt__(self, other):
		return self.g + self.h < other.g + other.h

	def __str__(self):
		if self.pre_node != None:
			return '[x=%s y=%s ang=%s g=%s h=%s pre_node=(%s, %s)]' % (self.x, self.y, self.ang, self.g, self.h, self.pre_node.x, self.pre_node.y)
		else:
			return '[x=%s y=%s ang=%s g=%s h=%s pre_node=None]' % (self.x, self.y, self.ang, self.g, self.h)

def in_list(node, l):
	if len(l) == 0:
		return False
	else:
		for n in l:
			if node.x == n.x and node.y == n.y and abs(node.ang - n.ang) <= 5:
				return True
	return False
		
class AStar(object):
	
	def __init__(self, start_point, targets, obstacle_map, fcn_maps, car):
		self.S = start_point
		self.Ts = targets
		self.T = targets[0]
		self.obstacle_map = obstacle_map
		self.obstacle_distance_map = []
		self.collision_r = car.hight / 2 / 0.2
		self.fcn_maps = fcn_maps
		self.fcn_map = fcn_maps[0]
		self.car = car
		self.lookup_table = generate_circle_table()
		self.lookup_dict = {}
		for i,e in enumerate(self.lookup_table):
			self.lookup_dict[e[2]] = i
		self.visualization_map = np.full((tc.MAP_ROWS, tc.MAP_COLS, 3), 255, np.uint8)
		self.visualize_obstacle_map(obstacle_map)
		for fcn_map in fcn_maps:
			self.visualize_prediction_map(fcn_map)
		for T in targets:
			cv2.circle(self.visualization_map, (T.y, T.x), 2, (31, 31, 197), 8, 1)
		cv2.circle(self.visualization_map, (self.S.y, self.S.x), 2, (171, 90, 0), 8, 1)
		self.searched = []

	def visualize_obstacle_map(self, obstacle_map):
		for row in range(tc.MAP_ROWS):
			if row >= obstacle_map.shape[0]:
				continue
			for col in range(tc.MAP_COLS):
				if col >= obstacle_map.shape[1]:
					continue
				if obstacle_map[row][col] != 0:
					self.visualization_map[row][col] = [46,36,32]

	def visualize_prediction_map(self, prediction_map):
		for row in range(tc.MAP_ROWS-1):
			for col in range(tc.MAP_COLS):
				if prediction_map[row][col] > 0.5:
					self.visualization_map[row][col] = [222,249,250]


	#@profile
	def get_next_nodes(self, node_now, car):
		next_nodes_all = []
		next_nodes_final = []
		ang_range = 0
		if car.speed < 2:
			ang_range = 45
		elif car.speed < 5:
			ang_range = 30
		elif car.speed < 10:
			ang_range = 15
		else:
			ange_range = 10
		ang_index = self.lookup_dict[node_now.ang]
		left_i = right_i = ang_index
		while 1:
			left_i -= 1
			step = self.lookup_table[left_i]
			ang_err = abs(node_now.ang - step[2])
			if not (ang_err <= ang_range or 360 - ang_err <= ang_range):
				break
			next_nodes_all.append(Node(node_now.x + step[0], node_now.y + step[1], step[2]))
		while 1:
			if right_i > 71:
				right_i -= 71
			step = self.lookup_table[right_i]
			right_i += 1
			ang_err = abs(node_now.ang - step[2])
			if not (ang_err <= ang_range or 360 - ang_err <= ang_range):
				break
			next_nodes_all.append(Node(node_now.x + step[0], node_now.y + step[1], step[2]))
		'''
		for step in self.lookup_table:
			ang_err = abs(node_now.ang - step[2])
			if ang_err < ang_range or 360 - ang_err < ang_range:
				next_nodes_all.append(Node(node_now.x + step[0], node_now.y + step[1], step[2]))
			if ang_err == 0 or (ang_range - ang_err < 5 and ang_range - ang_err > 0) or (ang_range - (360 - ang_err) < 5 and ang_range - (360 - ang_err) > 0):
				next_nodes_final.append(Node(node_now.x + step[0], node_now.y + step[1], step[2]))
		#print('sampling space size:', len(next_nodes_all))
		next_nodes_final += random.sample(next_nodes_all, max(int(len(next_nodes_all)/4), 3))
		#print('get %s sub_nodes' % len(next_nodes_final))
		'''
		#return next_nodes_final
		return next_nodes_all

	def cost(self, node_now, node_next):
		return sqrt((node_next.x - node_now.x)**2+(node_next.y - node_now.y)**2) + (1 / 10) * abs(node_next.ang - node_now.ang)

	def heuristic(self, node_next):
		return 2 * sqrt((self.T.x - node_next.x)**2 + (self.T.y - node_next.y)**2) + (1 / 20) * abs(node_next.ang - self.T.ang)

	def is_goal(self, node):
		ang_err = abs(self.T.ang - node.ang)
		if abs(self.T.x - node.x) <= 3 and abs(self.T.y - node.y) <= 3 and (ang_err <= 20 or 360 - ang_err <= 20):
			self.T = node
			return True
		else:
			return False

	def get_distance_map(self, _map):
		dx = (0, 0, -1, 1, -1, -1, 1, 1)
		dy = (1, -1, 0, 0, 1, -1, 1, -1)
		dis = (1, 1, 1, 1, 1.41, 1.414, 1.414, 1.414)
		tmp_list = []
		for row in range(tc.MAP_ROWS):
			if row >= _map.shape[0]:
				continue
			for col in range(tc.MAP_COLS):
				if col >= _map.shape[1]:
					continue
				if _map[row][col] > 0:
					tmp_list.append([row, col])
		head = 0
		tail = len(tmp_list)
		distance_map = [[10**8]*tc.MAP_COLS for i in range(tc.MAP_ROWS)]
		for p in tmp_list:
			distance_map[p[0]][p[1]] = 0
		while head != tail:
			p = tmp_list[head]
			head += 1
			for d in range(len(dx)):
				p_new = [p[0]+dx[d], p[1]+dy[d]]	
				if p_new[0] < 0 or p_new[0] >= tc.MAP_ROWS or p_new[1] < 0 or p_new[1] >= tc.MAP_COLS or distance_map[p_new[0]][p_new[1]] < 10**8:
					continue
				distance_map[p_new[0]][p_new[1]] = distance_map[p[0]][p[1]] + dis[d]
				tmp_list.append(p_new)
				tail += 1
		return distance_map

	def collision_check(self, node_start, node_end):
		if self.obstacle_distance_map[node_end.x][node_end.y] <= self.collision_r:
			return True
		if self.obstacle_distance_map[int((node_end.x+node_start.x)/2)][int((node_end.y+node_start.y)/2)] <= self.collision_r:
			return True
		return False
		pass
		delta_x = node_end.x - node_start.x
		delta_y = node_end.y - node_start.y
		abs_delta_x = abs(delta_x)
		abs_delta_y = abs(delta_y)
		if abs_delta_x >= abs_delta_y:
			k = abs_delta_y / abs_delta_x
			inter_node_x = inter_node_y = 0
			for dx in range(abs_delta_x):
				inter_node_x = int(node_start.x + (delta_x/abs_delta_x) * dx)
				if delta_y == 0:
					inter_node_y = node_start.y
				else:
					inter_node_y = int(node_start.y + (delta_y/abs_delta_y) * k * dx)
				if self.obstacle_distance_map[inter_node_x][inter_node_y] <= self.collision_r:
					return True
		else:
			k = abs_delta_x / abs_delta_y
			inter_node_x = inter_node_y = 0
			for dy in range(abs_delta_y):
				inter_node_y = int(node_start.y + (delta_y/abs_delta_y) * dy)
				if delta_x == 0:
					inter_node_x = node_start.x
				else:
					inter_node_x = int(node_start.x + (delta_x/abs_delta_x) * k * dy)
				if self.obstacle_distance_map[inter_node_x][inter_node_y] <= self.collision_r:
					return True
		return False
	
	def visualization(self):
		cv2.imshow('path planning', self.visualization_map)
		cv2.waitKey(1)

	def path_visualization(self, path):
		for i in range(len(path)-1):
			cv2.line(self.visualization_map, (path[i].y, path[i].x), (path[i+1].y, path[i+1].x), (29, 147, 248), 1, 8)
		self.visualization()

	def get_path_by_pre_node(self, node):
		path = []
		pre = node
		while pre != None:
			path.append(pre)
			pre = pre.pre_node
		path.reverse()
		return path

	def clear_searched(self):
		#print('searched list size:', len(self.searched))
		for p in self.searched:
			self.visualization_map[p[0]][p[1]] = [255,255,255]
		
	#@profile
	def search(self):
		self.obstacle_distance_map = self.get_distance_map(self.obstacle_map)
		self.S.h = self.heuristic(self.S)
		success_num = 0
		failed_num = 0
		for t in range(len(self.Ts)):
			self.T = self.Ts[t]
			self.fcn_map = self.fcn_maps[t]
			if VISUALIZE:
				self.clear_searched()
			open_list_array = np.zeros([tc.MAP_ROWS, tc.MAP_COLS, 72], dtype=np.bool)
			close_list_array = np.zeros([tc.MAP_ROWS, tc.MAP_COLS, 72], dtype=np.bool)
			open_list = []
			self.searched = []
			heappush(open_list, self.S)
			open_list_array[self.S.x][self.S.y][int(self.S.ang/5)] = True
			steps = 0
			start_time = datetime.now().timestamp()
			while len(open_list) > 0:
				steps += 1
				node_now = heappop(open_list)
				if self.is_goal(node_now) or steps > 15000:
					break
				close_list_array[node_now.x][node_now.y][int(node_now.ang/5)] = True
				#print('----------------------------')
				steps += 1
				#print('the '+str(steps)+' step expanded node:', node_now)
				self.visualization_map[node_now.x][node_now.y] = [110, 214, 179]
				if VISUALIZE:
					self.searched.append([node_now.x, node_now.y])
					self.visualization()
				for node_next in self.get_next_nodes(node_now, self.car):
					#print('next_node:', node_next)
					#path should be in the map
					if node_next.x < 0 or node_next.y < 0 or node_next.x >= tc.MAP_ROWS or node_next.y >= tc.MAP_COLS:
						continue
					#path should ensure the safe of car
					if self.collision_check(node_now, node_next):
						continue
					if close_list_array[node_next.x][node_next.y][int(node_next.ang/5)] or self.obstacle_map[node_next.x][node_next.y] != 0:
						continue
					w = min(2, 0.3 - log(self.fcn_map[node_next.x][node_next.y]))
					#w = 1
					g_new = node_now.g + w * self.cost(node_now, node_next)
					if not open_list_array[node_next.x][node_next.y][int(node_next.ang/5)]:
						node_next.pre_node = node_now
						#calculate node_next g and h value
						node_next.g = g_new
						node_next.h = w * self.heuristic(node_next)
						heappush(open_list, node_next)
						open_list_array[node_next.x][node_next.y][int(node_next.ang/5)] = True
					else:
						#compare the new way with the old one
						if g_new < node_next.g:
							node_next.g = g_new
							node_next.pre_node = node_now
						else:
							continue
			end_time = datetime.now().timestamp()
			searching_time = (end_time - start_time) * 1000
			print('AStar searching in %sms' % searching_time)
			print('AStar searching steps:', steps)
			if self.T.pre_node != None:
				success_num += 1
				print('successfully find the path')
				#get the planned path
				planned_path = self.get_path_by_pre_node(self.T)
				if VISUALIZE:
					self.path_visualization(planned_path)
			else:
				failed_num += 1
				print('failed to find the path')
		print('the successful number:', success_num)
		print('the failed number:', failed_num)
		print('successful rate:', success_num/(failed_num+success_num))
		if VISUALIZE:
			cv2.waitKey(0)
