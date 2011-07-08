from math import exp

# First order system
class FirstOrder:
	"""Simulate a first order response"""

	def __init__(self, period, tau, k = 1):
		"""Initialize data
		period : sampling period
		tau : system time constant"""
		self.y = 0
		self.k = k
		self.E = exp(-period/tau)

	def process(self, c):
		"""Compute the system response
		c : command to apply"""
		c *= self.k
		self.y = c + (self.y - c) * self.E
		return self.y

# Integral
class Integral:
	"""Simple numerical integration"""

	def __init__(self, dt):
		"""Initialize the integration
		dt : sampling period"""
		self.dym1 = 0
		self.dt_2 = dt / 2.0
		self.y = 0
	
	def process(self, dy):
		"""Compute the system response
		dy : element to integrate"""
		self.y += (self.dym1 + dy) * self.dt_2
		return self.y

# Derivative
class Derivative:
	"""Simple numerical derivative"""

	def __init__(self, dt):
		"""Initialize the derivative
		dt : sampling period"""
		self.iym1 = 0
		self.dt = dt
		self.y = 0
	
	def process(self, iy):
		"""Compute the system response
		iy : element to differentiate"""
		self.y = (iy - self.iym1) / self.dt
		self.iym1 = iy
		return self.y


