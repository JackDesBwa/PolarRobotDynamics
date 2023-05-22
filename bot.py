#!/usr/bin/env python3

# Libraries
from math import sin, cos, pi
from random import random
from user import control
from tool import FirstOrder, Integral, Derivative
import simu_config

if simu_config.display_plots:
	try:
		import matplotlib.pylab as plt
	except ModuleNotFoundError:
		print('Plots not possible, please install matplotlib')
		simu_config.display_plots = False

if simu_config.display_live:
	try:
		import pyglet
	except ModuleNotFoundError:
		print('Live view not possible, please install pyglet')
		simu_config.display_live = False

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

dt = simu_config.dt
robot = System(dt)
plots = []
deltat = simu_config.total_time
kmax = int(deltat / dt)
dtheta = Derivative(dt)
dcurv = Derivative(dt)

if simu_config.display_live:
	window = pyglet.window.Window()
	batch = pyglet.graphics.Batch()
	img_robot = pyglet.image.load('robot.png')
	img_robot.anchor_x = img_robot.width // 2
	img_robot.anchor_y = img_robot.height // 2 - 6
	pyg_robot = pyglet.sprite.Sprite(img_robot, x=window.width/2, y=window.height/2, batch=batch)
	gap = 25
	label_t = pyglet.text.Label(text="t: 0", batch=batch, x=10, y=window.height-10-gap*1)
	label_x = pyglet.text.Label(text="x: 0", batch=batch, x=10, y=window.height-10-gap*2)
	label_y = pyglet.text.Label(text="y: 0", batch=batch, x=10, y=window.height-10-gap*3)
	label_th = pyglet.text.Label(text="th: 0", batch=batch, x=10, y=window.height-10-gap*4)

	@window.event
	def on_draw():
		window.clear()
		batch.draw()

t = 0
def update(dt):
	global t
	t +=  dt
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
	if simu_config.display_live:
		pyg_robot.rotation = -theta * 180 / pi
		pyg_robot.x = window.width/2 + x * 1000
		pyg_robot.y = window.height/2 + y * 1000
		label_t.text = 't: ' + str(round(t, 2))
		label_x.text = 'x: ' + str(round(x * 1000)) + "mm"
		label_y.text = 'y: ' + str(round(y * 1000)) + "mm"
		label_th.text = 'th: ' + str(round(theta*180/pi)) + "°"
	if t > kmax * dt:
		window.close()

if simu_config.display_live:
	pyglet.clock.schedule_interval(update, dt)
	pyglet.app.run()
	display_live = False

if len(plots) < kmax:
	for k in range(len(plots), kmax):
		update(dt);

if simu_config.display_plots:
	color1 = '#3771c8'
	color1g = '#aab6c8'
	color2 = '#7337c8'
	plt.figure(figsize=(10,12))
	axes_t_tics = plt.subplot(421)
	plt.title('Left wheel speed (tics/s)')
	plt.plot(tuple(plots[i][0] for i in range(kmax)), tuple(plots[i][3] for i in range(kmax)), color=color1)
	if simu_config.plot_otherspeed:
		plt.plot(tuple(plots[i][0] for i in range(kmax)), tuple(plots[i][4] for i in range(kmax)), color=color1g)
	plt.subplot(422, sharex=axes_t_tics, sharey=axes_t_tics)
	plt.title('Right wheel speed (tics/s)')
	plt.plot(tuple(plots[i][0] for i in range(kmax)), tuple(plots[i][4] for i in range(kmax)), color=color1)
	if simu_config.plot_otherspeed:
		plt.plot(tuple(plots[i][0] for i in range(kmax)), tuple(plots[i][3] for i in range(kmax)), color=color1g)
	plt.subplot(423)
	plt.title('XY (mm)')
	plt.plot(tuple(plots[i][1] * 1000 for i in range(kmax)), tuple(plots[i][2] * 1000 for i in range(kmax)), color=color1)
	plt.axis('equal')
	plt.subplot(424, sharex=axes_t_tics)
	plt.title('Commands (%)')
	plt.plot(tuple(plots[i][0] for i in range(kmax)), tuple(plots[i][5] * 100 for i in range(kmax)), color=color1)
	plt.plot(tuple(plots[i][0] for i in range(kmax)), tuple(plots[i][6] * 100 for i in range(kmax)), color=color2)
	plt.subplot(425, sharex=axes_t_tics)
	plt.title('Angle (deg)')
	plt.plot(tuple(plots[i][0] for i in range(kmax)), tuple(plots[i][7] * 180 / pi for i in range(kmax)), color=color1)
	plt.subplot(426, sharex=axes_t_tics)
	plt.title('Curviligne distance (mm)')
	plt.plot(tuple(plots[i][0] for i in range(kmax)), tuple(plots[i][8] * 1000 for i in range(kmax)), color=color1)
	plt.subplot(427, sharex=axes_t_tics)
	plt.title('X (mm)')
	plt.plot(tuple(plots[i][0] for i in range(kmax)), tuple(plots[i][1] * 1000 for i in range(kmax)), color=color1)
	plt.subplot(428, sharex=axes_t_tics)
	plt.title('Y (mm)')
	plt.plot(tuple(plots[i][0] for i in range(kmax)), tuple(plots[i][2] * 1000 for i in range(kmax)), color=color1)

	if simu_config.plot_states:
		plt.figure()
		plt.subplots_adjust(left=0.1, bottom=0.05, right=0.95, top=0.95, wspace=0.3, hspace=0.4)
		plt.subplot(321)
		plt.title('curv / dcurv')
		lt.plot(tuple(plots[i][8] for i in range(kmax)), tuple(plots[i][10] for i in range(kmax)))
		lt.subplot(322)
		lt.title('theta / dtheta')
		lt.plot(tuple(plots[i][7] * 180 / pi for i in range(kmax)), tuple(plots[i][9] * 180 / pi for i in range(kmax)))
		lt.subplot(323)
		lt.title('curv / theta')
		lt.plot(tuple(plots[i][8] for i in range(kmax)), tuple(plots[i][7] * 180 / pi for i in range(kmax)))
		lt.subplot(324)
		lt.title('dcurv / dtheta')
		lt.plot(tuple(plots[i][10] for i in range(kmax)), tuple(plots[i][9] * 180 / pi for i in range(kmax)))
		lt.subplot(325)
		lt.title('theta / dcurv')
		lt.plot(tuple(plots[i][7] * 180 / pi for i in range(kmax)), tuple(plots[i][10] for i in range(kmax)))
		lt.subplot(326)
		lt.title('curv / dtheta')
		lt.plot(tuple(plots[i][8] for i in range(kmax)), tuple(plots[i][9] * 180 / pi for i in range(kmax)))

	plt.tight_layout()
	plt.show()
