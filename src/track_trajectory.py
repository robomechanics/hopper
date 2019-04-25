import odrive
from odrive.enums import *
import time
import numpy as np
from matplotlib import pyplot as plt
import argparse

def counts_to_radians(counts):
	return counts*2*np.pi/8192

# Odrive setup
def main():
	parser = argparse.ArgumentParser(description='Trajectory Tracking')
	parser.add_argument('--hardware', action="store_true", dest='use_hardware', default=False)
	results = parser.parse_args()
	if (results.use_hardware):
		print("Sending commands to odrive")
	else:	
		print("Plotting commands only")

	if (results.use_hardware):
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
		time.sleep(3)

	t0 = time.perf_counter()

	t = time.perf_counter() - t0


	# Trajectory features
	w_hz = 0.5 #frequency of sine wave in hz
	w = 2 * np.pi * w_hz
	A = 2000 #amplitude of sine wave in counts

	# Motor setup properties
	I_rotor = 0.00008 #m^2*kg, measured from CAD and scaled by mass ratio (rotor inertia only)
	I_load = 0.0001351255 + 0.000000375 + .001296 # rod, attachment screws, added weight m^2*kg
	kv = 135
	kt = 1/(kv*2*np.pi/60)


	pds = np.array([])
	vds = np.array([])
	ads = np.array([])
	tauds = np.array([])
	cds = np.array([])
	ps = np.array([])
	ts = np.array([])

	while(t < 2):
		t = time.perf_counter() - t0

		# Compute desired values from desired trajectory
		p_desired = A * np.sin(w * t)
		v_desired = A * w * np.cos(w * t)
		a_desired = - counts_to_radians(A) * w * w * np.sin(w*t)

		# Transform acceleration to current
		tau_desired = I_load * a_desired
		cur_desired = tau_desired/kt

		
		if (results.use_hardware):
			# Update setpoints
			odrv0.axis0.controller.pos_setpoint = p_desired
			odrv0.axis0.controller.vel_setpoint = v_desired
			odrv0.axis0.controller.current_setpoint = cur_desired

			# get pos estimate
			p = odrv0.axis0.encoder.pos_estimate
		
		pds = np.append(pds,p_desired)
		vds = np.append(vds,v_desired)
		ads = np.append(ads,a_desired)
		tauds = np.append(tauds, tau_desired)
		cds = np.append(cds, cur_desired)
		if (results.use_hardware): ps = np.append(ps,p)
		ts = np.append(ts,t)


	if (results.use_hardware):
		odrv0.axis0.requested_state = AXIS_STATE_IDLE
		np.savetxt('../data/ts_6hz_pv.csv', ts)
		np.savetxt('../data/ps_6hz_pv.csv', ps)
		np.savetxt('../data/pds_6hz_pv.csv', pds)

		plt.plot(ts, pds - ps) # desired position plot
		#plt.plot(ts, ps) # measured position plot
		plt.show()
	else:
		plt.plot(ts, pds) # desired position plot
		plt.plot(ts, vds) # desired velocity plot
		plt.plot(ts, ads) # desired acceleration plot
		plt.plot(ts, tauds) # desired torque plot
		plt.plot(ts, cds) # desired current plot
		plt.show()


	

if __name__ == "__main__": main()