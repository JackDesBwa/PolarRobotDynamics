# User control
def control(x, y, vl, vr, theta, curv, dtheta, dcurv):
	""" User control function
	x : X coordonate
	y : Y coordonate
	vl : left wheel velocity
	vr : right wiheel velocity
	theta : angular heading
	curv : curvilinear distance
	dtheta : angular rate
	dcurv : curvilinear speed
	@return : (cmd_left, cmd_right)
	cmd_left : left motor command between -1 and 1
	cmd_right : right motor command between -1 and 1
	"""
	
	cmd_left  = 0.2 # 20% of max speed for the left motor
	cmd_right = 0.2 # 20% of max speed for the right motor
	
	return (cmd_left, cmd_right) 
