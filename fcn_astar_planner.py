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
		prediction_path = predictions_file_path+str(i)+'.png'
		input_img = cv2.imread(input_path, 1)
		prediction_img = cv2.imread(prediction_path, 0)
		inputs.append(input_img)
		predictions.append(prediction_img)
	#get the car status
	car  = Car()
	car.speed = 1 #m/s
	print('car:', car)
	#extract the start point, target point and obstacle map
	start_point = Node(300, 75, 0)
	target_point = get_target(inputs[0])
	obstacle_map = inputs[0][:,:,2]
	#obstacle_map = np.zeros([400, 152])
	#create AStar search
	astar = AStar(start_point, target_point, obstacle_map, predictions[0], car)
	astar.search()
	'''
	cv2.imshow('obstacle map', obstacle_map)
	#cv2.imshow('prediction', predictions[0])
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	'''

