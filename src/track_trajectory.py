import odrive
from odrive.enums import *
import time
import numpy as np
from matplotlib import pyplot as plt


# Odrive setup
print("Finding an odrive...")
odrv0 = odrive.find_any()
print("Connected.")

print("Current bus voltage is " + str(odrv0.vbus_voltage) + "V")

time.sleep(1)

print("Using prior calibration stored in configuration for Axis0.")

print("Starting closed loop control...")
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("Closed loop control.")

print("Moving to position 0")
odrv0.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
odrv0.axis0.controller.pos_setpoint = 0


# Feedforward Term



time.sleep(2)

t0 = time.perf_counter()

t = time.perf_counter() - t0


# Trajectory features
w_hz = 2 #frequency of sine wave in hz
w = 2 * np.pi * w_hz
A = 2000 #amplitude of sine wave in counts


# Tunable properties
I_load = 0.1 #m^2*kg
kv = 135
kt = 8.27/kv


pds = np.array([])
ps = np.array([])
ts = np.array([])


while(t < 10):
	t = time.perf_counter() - t0

	# Compute desired values from desired trajectory
	p_desired = A * np.sin(w * t)
	v_desired = A * w * np.cos(w * t)
	a_desired = -A * w^2 * np.sin(w*t)

	# Transform acceleration to current
	tau = I_load * a_desired
	cur_desired = tau/kt

	# Update setpoints 
	odrv0.axis0.controller.pos_setpoint = p_desired
	odrv0.axis0.controller.vel_setpoint = v_desired
	odrv0.axis0.controller.current_setpoint = 0

	p = odrv0.axis0.encoder.pos_estimate
	#print("Position Estimate: ", p)
	#print("Position Desired: ", p_desired)
	pds = np.append(pds,p_desired)
	ps = np.append(ps,p)
	ts = np.append(ts,t)

odrv0.axis0.requested_state = AXIS_STATE_IDLE

#np.savetxt('../data/ts_2hz_p.csv', ts)
#np.savetxt('../data/ps_2hz_p.csv', ps)
#np.savetxt('../data/pds_2hz_p.csv', pds)
