import odrive
from odrive.enums import *
import time
import numpy as np
from scipy import interpolate
from matplotlib import pyplot as plt
import argparse

class struct(object): #include this from somewhere else
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

class Hopper(object):
	def __init__(self, raw_traj, ctrl_frequency = 200, new_calibration = False): # raw_traj should be 4 x n (time s, theta rads, thetadot rad/s, torque n-m)
		self.odrv0 = None
		self.ctrl_frequency = ctrl_frequency

		# Motor properties
		self.motor = struct(kt = 0.07) #this is not the exact value for kt, ballpark only

		# Encoder properties
		self.encoder = struct(offset = 0)

		# Search for odrive
		self.find_odrive()

		# Calibrate odrive if necessary
		if (new_calibration):
			self.calibration(self.odrv0)

		time.sleep(1)

		# convert trajectory
		traj_math_units = self.offline_convert_traj_to_fixed_timestep(traj_raw) #traj is now linearly interpolated and sampled at our control frequency, but units are still wrong
		traj = self.offline_convert_traj_units(traj_math_units) #traj is now 4xn, with correct odrive units (seconds, counts, counts/s, amps)

		# Wait for key to begin
		print("Hopper waiting for command (enter to begin trajectory)")
		waitkey = input()

		print("Beginning trajectory...")
		self.track_trajectory(traj)

	def find_odrive(self):
		self.odrv0 = odrive.find_any()
		print("ODrive found. Current bus voltage is " + str(self.odrive.vbus_voltage) + "V.")

	def odrive_calibration(self, odrv):
		odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION

	def radians_to_counts(self, rads):
		return rads * 8192.0/(2*np.pi)

	def counts_to_radians(self, counts):
		return counts*2*np.pi/8192.0

	def calibration(self, odrv):
		time.sleep(1)
		print("Configuring Axis (T-motor kv135)...")
		odrv.axis0.motor.config.current_lim = 20
		odrv.axis0.controller.config.vel_limit = 30000
		odrv.axis0.motor.config.pole_pairs = 21
		odrv.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
		odrv.axis0.encoder.config.cpr = 8192
		odrv.axis0.controller.config.vel_limit = 8192 * 15 # 15 revs/s
		print("Axis configured.")

		time.sleep(1)
		print("Starting calibration sequence on Axis0...")
		odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
		while odrv.axis0.current_state != AXIS_STATE_IDLE:
			time.sleep(0.05)
		print("Axis0 calibrated.")

		odrv.save_configuration()
		print("Configuration saved. Rebooting... (may need to reconnect)")
		odrv.reboot()


	def offline_convert_traj_to_fixed_timestep(self, raw_traj):
		#raw_traj is 4xn
		assert(traj.shape[0] == 4)
		tf = traj[1,-1] #last time index in traj

		traj = np.array([])

		tstep = 1.0/self.ctrl_frequency
		t = 0

		t_sampled = np.arange(0,tf,tstep)
		Q = np.vstack(raw_traj[1,:], raw_traj[2,:], raw_traj[3,:]) # reshape our trajectory without the times for linear interpolation

		f_sampled = interp1d(raw_traj[0,:],Q) # create linear interpolation function based on raw_traj timesteps

		Q_sampled = f_interp(t_sampled) # sample our linear interpolation function at desired sample times

		traj_math_units = np.vstack(t_sampled, Q_sampled[0,:], Q_sampled[1,:], Q_sampled[2,:]) # reshape our linearly interpolated values back into traj form

		return traj_math_units


	def offline_convert_traj_units(self, traj_math_units):
		# row 0 goes from s        -> s           (no conversion)
		# row 1 goes from counts   -> radians     (be mindful of encoder count offset here)
		# row 2 goes from counts/s -> radians/s
		# row 3 goes from n-m      -> amps

		row0 = traj_math_units[0,:] # no conversion
		row1 = self.counts_to_radians(traj_math_units[1,:] - self.encoder.offset)
		row2 = self.counts_to_radians(traj_math_units[2,:])
		row3 = traj_math_units[3,:]/self.motor.kt # TODO: experimentally determine better motor model

		# TODO check that commanded currents (row3) don't excede present current limits

		traj = np.vstack(row0, row1, row2, row3)
		return traj


	def track_trajectory(self, odrv):
		# traj should be 4 x n [time, theta, thetadot, current] in units s, rads, rads/s and amps respectively

		assert(traj.shape[0] == 4)


		tstep = 1/self.ctrl_frequency #timestep in seconds
		n = traj.shape[1] # number of steps in simulation

		
		t0 = time.perf_counter()

		for i in range(0, n):
			
			t_sampled_at = traj[0,i] #assert that this is close to our actual (measured) time at this point in the loop

			# Set desired p and d as well as open loop gains
			odrv0.axis0.controller.pos_setpoint = traj[1,i] # position in counts
			odrv0.axis0.controller.vel_setpoint = traj[2,i] # torque in counts/s
			odrv0.axis0.controller.current_setpoint = traj[3,i] # current in amps

			while(time.perf_counter() - t0 < i*tstep): #dont continue until tset has passed relative to absolute time (t0)
				pass
			

			
		






		


