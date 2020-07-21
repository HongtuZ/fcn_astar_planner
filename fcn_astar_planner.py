import cv2
import numpy as np
from math import *
from queue import PriorityQueue
from tiev_astar import *
from car import Car
#step1:read the fcn input and outputs
#step2:extract the start point and targets from input image
#step3:redesign the planning map with output images


def get_target(input_img):
	target_channel = input_img[:,:,0]
	target_x = 0
	target_y = 0
	counter = 0
	for row in range(target_channel.shape[0]):
		for col in range(target_channel.shape[1]):
			if target_channel[row,col] != 0:
				target_x += row
				target_y += col
				counter += 1
	target_x /= counter
	target_y /= counter
	return Node(int(target_x), int(target_y))

if __name__ == '__main__':
	#read the inputs and outputs
	inputs_file_path = 'inputs/'
	predictions_file_path = 'predictions/'
	inputs = []
	predictions = []
	for i in range(1):
		input_path = inputs_file_path+str(i)+'.png'
		prediction_path = predictions_file_path+str(i)+'.txt'
		input_img = cv2.imread(input_path, 1)
		inputs.append(input_img)
		prediction = np.loadtxt(prediction_path)
		predictions.append(prediction)
	#get the car status
	car  = Car()
	car.speed = 1 #m/s
	print('car:', car)
	#extract the start point, target point and obstacle map
	start_point = Node(300, 75, 0)
	target_point = get_target(inputs[0])
	obstacle_map = inputs[0][:,:,2]
	prediction_map = predictions[0]
	print('input size:', obstacle_map.shape)
	#obstacle_map = np.zeros([400, 152])
	#create AStar search
	astar = AStar(start_point, target_point, obstacle_map, prediction_map, car)
	astar.search()
	'''
	cv2.imshow('obstacle_map', obstacle_map)
	prediction_img = np.full([predictions[0].shape[0], predictions[0].shape[1], 3], 0, np.uint8)
	for r in range(prediction_img.shape[0]):
		for c in range(prediction_img.shape[1]):
			if predictions[0][r][c] > 0:
				prediction_img[r][c] = [0, int(255*predictions[0][r][c]), 0]
	cv2.imshow('prediction', prediction_img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	'''

