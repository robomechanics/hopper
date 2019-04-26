import odrive
from odrive.enums import *
import time

print("Finding an odrive...")
odrv0 = odrive.find_any()
print("Connected.")

print("Current bus voltage is " + str(odrv0.vbus_voltage) + "V")

time.sleep(1)
print("Configuring Axis0 (T-motor kv135)...")
odrv0.axis0.motor.config.current_lim = 20
odrv0.axis0.controller.config.vel_limit = 30000
odrv0.axis0.motor.config.pole_pairs = 21
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.encoder.config.cpr = 8192
odrv0.axis0.controller.config.vel_limit = 8192 * 15 # 15 revs/s

print("Axis0 configured.")

time.sleep(1)
print("Starting calibration sequence on Axis0...")
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while odrv0.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.05)
print("Axis0 calibrated.")

odrv0.save_configuration()

print("Configuration saved")