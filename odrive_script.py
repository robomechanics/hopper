import odrive
import time

print("Finding an odrive...")
odrv0 = odrive.find_any()
print("Connected.")

print("Starting calibratiom sequence on axis0...")
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while odrv0.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.05)
print("Axis0 calibrated.")

print("Starting closed loop control...")
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("Closed loop control.")

print("Current bus voltage is " + str(odrv0.vbus_voltage) + "V")
