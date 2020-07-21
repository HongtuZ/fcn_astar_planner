class Car(object):
	def __init__(self):
		self.length = 5
		self.width = 2
		self.hight = 2
		self.speed = 0
		self.steering_wheel = 0

	def __str__(self):
		return 'Length:%sm Width:%sm Hight:%sm Speed:%sm/s Steer:%sdegree' % (self.length, self.width, self.hight, self.speed, self.steering_wheel)
