import odrive
from odrive.enums import *
import time
import numpy as np
from matplotlib import pyplot as plt
import argparse

class Hopper(object):
	def __init__(self):
		self.odrive = None

		# Search for odrive
		find_odrive()

	def find_odrive(self):
		self.odrive = odrive.find_any()

	def odrive_calibration(self):
		self.odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION

	def radians_to_counts(self, rads):
		return rads * 8192/(2*np.pi)

	def counts_to_radians(self, counts):
		return counts*2*np.pi/8192
		


