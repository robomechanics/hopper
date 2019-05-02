import odrive
from odrive.enums import *
import time
import numpy as np
from scipy.interpolate import interp1d
from matplotlib import pyplot as plt

class struct(object): #include this from somewhere else
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

class Hopper(object):
	def __init__(self, raw_traj = [], ctrl_frequency = 200, new_calibration = False): # raw_traj should be 4 x n (time s, theta rads, thetadot rad/s, torque n-m)
		self.odrv0 = None
		self.ctrl_frequency = ctrl_frequency

		# Motor properties
		self.motor = struct(kt = 0.07) #this is not the exact value for kt, ballpark figure

		# Encoder properties
		self.encoder = struct(offset = 0, cpr = 8192)

		# Hardcoded jump properties
		self.jump = struct(theta_start = np.pi/9, theta_middle = np.pi/2, theta_end = 2*np.pi/3, t_retract = 0.2)
		self.traj = None

		# convert trajectory if one was passed in
		if (raw_traj != []):
			self.traj = self.offline_convert_traj(raw_traj) #traj is now linearly interpolated and sampled at our control frequency, units corrected
		
		print("Hopper object initialized. ")

	def find_odrive(self):
		print("Finding odrive...")
		self.odrv0 = odrive.find_any()
		print("ODrive found. Current bus voltage is " + str(self.odrive.vbus_voltage) + "V.")


	# define various conversions between radians and encoder for leg
	def odrive_calibration(self, odrv):
		odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION

	def radians_per_sec_to_counts_per_sec(self, rads_per_sec):
		return rads_per_sec * 8192.0/(2*np.pi)

	def counts_per_sec_to_radians_per_sec(self, counts_per_sec):
		return (counts_per_sec - self.encoder.offset) * 2 * np.pi / self.encoder.cpr

	def radians_to_counts(self, rads):
		return (rads * self.encoder.cpr/(2*np.pi)) - self.encoder.offset

	def counts_to_radians(self, counts):
		return (counts - self.encoder.offset) * 2 * np.pi / self.encoder.cpr

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


	def offline_convert_traj(self, raw_traj):
		#raw_traj is 4xn
		assert(raw_traj.shape[0] == 4)
		tf = raw_traj[1,-1] #last time index in traj

		traj = np.array([])

		tstep = 1.0/self.ctrl_frequency
		t = 0

		t_sampled = np.arange(0,tf,tstep)
		Q = np.vstack([raw_traj[1,:], raw_traj[2,:], raw_traj[3,:]]) # reshape our trajectory without the times for linear interpolation

		f_sampled = interp1d(raw_traj[0,:],Q) # create linear interpolation function based on raw_traj timesteps

		Q_sampled = f_sampled(t_sampled) # sample our linear interpolation function at desired sample times

		""" Convert to correct units for passing to odrive
		row 0 goes from s        -> s           (no conversion)
		row 1 goes from counts   -> radians     (be mindful of encoder count offset here)
		row 2 goes from counts/s -> radians/s
		row 3 goes from n-m      -> amps
		"""
		row0 = t_sampled #no_conversion
		row1 = self.counts_to_radians(Q_sampled[0,:])
		row2 = self.counts_per_sec_to_radians_per_sec(Q_sampled[1,:])
		row3 = Q_sampled[2,:]/self.motor.kt # TODO: experimentally determine better motor model

		return np.vstack([row0, row1, row2, row3])

	def odrv_dump_errors(self):
		dump_errors(self.odrv0, True)

	# hardcoded jump for meeting of the minds
	def jump(self):
		# go to start position
		print("Starting closed loop control...")
		self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		print("Closed loop control.")

		print("Starting position control mode...")
		self.odrv0.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
		print("Position control mode...")

		print("Moving to start positon...")


	def steady_state_jump(self):
		print("Starting closed loop control...")
		self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		print("Closed loop control.")

		print("Moving to start position")
		self.odrv0.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
		odrv0.axis0.controller.pos_setpoint = self.radians_to_counts(self.jump.theta_start)# position in counts

		pass

	# hardcoded double_jump for meeting of the minds
	def double_jump(self):
		# go to start position
		pass


	def track_trajectory(self, odrv):
		# self.traj should be 4 x n [time, theta, thetadot, current] in units s, rads, rads/s and amps respectively
		assert(self.traj != None)
		assert(self.traj.shape[0] == 4)
		n = self.traj.shape[1] # number of steps in simulation

		tstep = 1/self.ctrl_frequency #timestep in seconds
		
		t0 = time.perf_counter()

		for i in range(0, n):
			
			t_sampled_at = self.traj[0,i] #assert that this is close to our actual (measured) time at this point in the loop

			# Set desired p and d as well as open loop gains
			odrv0.axis0.controller.pos_setpoint = self.traj[1,i] # position in counts
			odrv0.axis0.controller.vel_setpoint = self.traj[2,i] # torque in counts/s
			odrv0.axis0.controller.current_setpoint = self.traj[3,i] # current in amps

			while(time.perf_counter() - t0 < i*tstep): #dont continue until tset has passed relative to absolute time (t0)
				pass
		






		


