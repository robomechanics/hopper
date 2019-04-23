import odrive
from odrive.enums import *
import time
import numpy as np
from matplotlib import pyplot as plt

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
odrv0.axis0.controller.pos_setpoint = 0

odrv0.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

t0 = time.perf_counter()
odrv0.axis0.controller.vel_setpoint = 5000

time.sleep(1)

alpha = 0.1
y = 0

t = time.perf_counter() - t0
ps = np.array([])
ts = np.array([])
xs = np.array([])
ys = np.array([])
while(t < 5):
	t = time.perf_counter() - t0
	x = odrv0.axis0.encoder.vel_estimate
	y = x * alpha + (1 - alpha) * y
	#p = odrv0.axis0.encoder.pos_cpr
	p = odrv0.axis0.encoder.pos_estimate
	print("Position Estimate: ", p)
	print("Velocity Estimate: ", x)
	print("Filtered Velocity Estimate: ", y)
	ps = np.append(ps,p)
	xs = np.append(xs,x)
	ys = np.append(ys,y)
	ts = np.append(ts,t)
	time.sleep(0.01)

odrv0.axis0.requested_state = AXIS_STATE_IDLE

ps_diff = np.diff(ps)
ts_diff = np.diff(ts)

vs_custom = np.divide(ps_diff,ts_diff)

np.savetxt('ts.csv', ts)
np.savetxt('ps.csv', ps)
np.savetxt('ys.txt', ys)

plt.plot(ts[:-1], vs_custom)
plt.show()



"""

