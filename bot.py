# Libraries
import numpy as np
import matplotlib.pylab as plt
from math import sin, cos, pi
from random import random
from user import control
from tool import FirstOrder, Integral, Derivative
import simu_config

# System
class System:
	def __init__(self, dt):
		"""Initialize the system
		dt : sampling period"""
		self.ml = FirstOrder(dt,
			simu_config.Tleft * (1 + (random() - 0.5)*2*simu_config.alea_Tleft),
			simu_config.Kleft * (1 + (random() - 0.5)*2*simu_config.alea_Kleft))
		self.mr = FirstOrder(dt,
			simu_config.Tright * (1 + (random() - 0.5)*2*simu_config.alea_Tright),
			simu_config.Kright * (1 + (random() - 0.5)*2*simu_config.alea_Kright))
		self.theta = Integral(dt)
		self.curv = Integral(dt)
		self.x = Integral(dt)
		self.y = Integral(dt)
		self.dt = dt
		self.ticspermeter = simu_config.ticspermeter
		self.diffticsperrad = simu_config.diffticsperrad
		
	
	def process(self, cl, cr):
		"""Compute the system response
		cl : command to apply to left motor
		cr : command to apply to right motor"""
		
		# Saturation
		if cl < -1: cl = -1
		if cr < -1: cr = -1
		if cl > 1: cl = 1
		if cr > 1: cr = 1

		# Motor process
		vl = self.ml.process(cl) # tics / s
		vr = self.mr.process(cr) # tics / s
		
		# Polar velocity
		vmean = (vl + vr) / 2 / self.ticspermeter # m/s
		vang = (vl - vr) / self.diffticsperrad    # rad/s
		
		# Angular and curvilign integration
		curv = self.curv.process(vmean)  # m
		theta = self.theta.process(vang) # rad
		
		# XY position
		self.x.process(vmean * cos(theta)) # m
		self.y.process(vmean * sin(theta)) # m
		
		return (x, y)

dt = simu_config.dt
robot = System(dt)
plots = []
deltat = simu_config.total_time
kmax = int(deltat / dt)
dtheta = Derivative(dt)
dcurv = Derivative(dt)

for k in xrange(kmax):
	t = k * dt
	x = robot.x.y
	y = robot.y.y
	vr = robot.mr.y
	vl = robot.ml.y
	theta = robot.theta.y
	curv = robot.curv.y
	dtheta.process(theta)
	dcurv.process(curv)
	cl, cr = control(x, y, vl, vr, theta, curv, dtheta.y, dcurv.y)
	robot.process(cl, cr)
	plots.append((t, x, y, vl, vr, cl, cr, theta, curv, dtheta.y, dcurv.y))

plt.subplot(421)
plt.title('Left wheel speed (tics/s)')
plt.plot(tuple(plots[i][0] for i in xrange(kmax)), tuple(plots[i][3] for i in xrange(kmax)))
if simu_config.plot_otherspeed:
	plt.plot(tuple(plots[i][0] for i in xrange(kmax)), tuple(plots[i][4] for i in xrange(kmax)))
plt.subplot(422)
plt.title('Right wheel speed (tics/s)')
plt.plot(tuple(plots[i][0] for i in xrange(kmax)), tuple(plots[i][4] for i in xrange(kmax)))
if simu_config.plot_otherspeed:
	plt.plot(tuple(plots[i][0] for i in xrange(kmax)), tuple(plots[i][3] for i in xrange(kmax)))
plt.subplot(423)
plt.title('XY (mm)')
plt.plot(tuple(plots[i][1] * 1000 for i in xrange(kmax)), tuple(plots[i][2] * 1000 for i in xrange(kmax)))
plt.axis([-250, 250, -250, 250])
plt.subplot(424)
plt.title('Commands (%)')
plt.plot(tuple(plots[i][0] for i in xrange(kmax)), tuple(plots[i][5] * 100 for i in xrange(kmax)))
plt.plot(tuple(plots[i][0] for i in xrange(kmax)), tuple(plots[i][6] * 100 for i in xrange(kmax)))
plt.subplot(425)
plt.title('Angle (deg)')
plt.plot(tuple(plots[i][0] for i in xrange(kmax)), tuple(plots[i][7] * 180 / pi for i in xrange(kmax)))
plt.subplot(426)
plt.title('Curvilinear distance (mm)')
plt.plot(tuple(plots[i][0] for i in xrange(kmax)), tuple(plots[i][8] * 1000 for i in xrange(kmax)))
plt.subplot(427)
plt.title('X (mm)')
plt.plot(tuple(plots[i][0] for i in xrange(kmax)), tuple(plots[i][1] * 1000 for i in xrange(kmax)))
plt.subplot(428)
plt.title('Y (mm)')
plt.plot(tuple(plots[i][0] for i in xrange(kmax)), tuple(plots[i][2] * 1000 for i in xrange(kmax)))

if simu_config.plot_states:
	plt.figure()
	plt.subplot(321)
	plt.title('curv / dcurv')
	plt.plot(tuple(plots[i][8] for i in xrange(kmax)), tuple(plots[i][10] for i in xrange(kmax)))
	plt.subplot(322)
	plt.title('theta / dtheta')
	plt.plot(tuple(plots[i][7] * 180 / pi for i in xrange(kmax)), tuple(plots[i][9] * 180 / pi for i in xrange(kmax)))
	plt.subplot(323)
	plt.title('curv / theta')
	plt.plot(tuple(plots[i][8] for i in xrange(kmax)), tuple(plots[i][7] * 180 / pi for i in xrange(kmax)))
	plt.subplot(324)
	plt.title('dcurv / dtheta')
	plt.plot(tuple(plots[i][10] for i in xrange(kmax)), tuple(plots[i][9] * 180 / pi for i in xrange(kmax)))
	plt.subplot(325)
	plt.title('theta / dcurv')
	plt.plot(tuple(plots[i][7] * 180 / pi for i in xrange(kmax)), tuple(plots[i][10] for i in xrange(kmax)))
	plt.subplot(326)
	plt.title('curv / dtheta')
	plt.plot(tuple(plots[i][8] for i in xrange(kmax)), tuple(plots[i][9] * 180 / pi for i in xrange(kmax)))

plt.show()
