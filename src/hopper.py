import odrive
from odrive.enums import *
import time
import numpy as np
from scipy import interpolate
from matplotlib import pyplot as plt
import argparse

class Hopper(object):
	def __init__(self, traj_raw, ctrl_frequency = 200, new_calibration = False): # traj_raw should be 4 x n (time s, theta rads, thetadot rad/s, torque n-m)
		self.odrv0 = None

		# Search for odrive
		self.find_odrive()

		# Calibrate odrive if necessary
		if (new_calibration):
			self.calibration(self.odrv0)

		time.sleep(1)

		# convert trajectory
		self.traj = self.offline_convert_traj_to_fixed_timestep(traj_raw,ctrl_frequency)

		# Wait for key to begin
		print("Hopper waiting for command (enter to begin trajectory)")
		waitkey = input()

		print("Beginning trajectory...")

	def find_odrive(self):
		self.odrv0 = odrive.find_any()
		print("ODrive found. Current bus voltage is " + str(self.odrive.vbus_voltage) + "V.")

	def odrive_calibration(self, odrv):
		odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION

	def radians_to_counts(self, rads):
		return rads * 8192/(2*np.pi)

	def counts_to_radians(self, counts):
		return counts*2*np.pi/8192

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


	def offline_convert_traj_to_fixed_timestep(self, traj_raw, ctrl_frequency):
		assert(traj.shape[0] == 4)
		tf = traj[1,-1] #last time index in traj

		traj = np.array([])

		tstep = 1.0/ctrl_frequency
		t = 0
		while(t < tf):



			t = t + tstep





	def track_trajectory(self, odrv, traj, tstep):
		# traj should be 3 x n [theta, thetadot, current] in units rads, rads/s and amps respectively

		assert(traj.shape[0] == 3)

		n = traj.shape[1] # number of steps in simulation

		
		t0 = time.perf_counter()

		for i in range(0, n):
			
			odrv0.axis0.controller.pos_setpoint = traj[1,i] # position in counts
			odrv0.axis0.controller.vel_setpoint = traj[2,i] # torque in counts/s
			odrv0.axis0.controller.current_setpoint = traj[3,i] # current in amps

			while(time.perf_counter() - t0 < i*tstep): #dont continue until tset has passed relative to absolute time (t0)
				pass
			

			
		






		


