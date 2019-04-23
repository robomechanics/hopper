import odrive
from odrive.enums import *
import time
import numpy as np
from matplotlib import pyplot as plt

CTRL_MODE_POSITION_CONTROL = 4
CTRL_MODE_FEEDFORWARD_CONTROL = 1

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

time.sleep(2)


t0 = time.perf_counter()

t = time.perf_counter() - t0

w = 2 #frequency of sine wave in hz
A = 2000 #amplitude of sine wave in counts

pds = np.array([])
ps = np.array([])
ts = np.array([])


while(t < 10):
	t = time.perf_counter() - t0
	p_desired = A * np.sin(2 * np.pi* w * t)
	v_desired = A * np.cos(2 * np.pi* w * t)
	odrv0.axis0.controller.pos_setpoint = p_desired
	#odrv0.axis0.controller.vel_setpoint = v_desired
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
