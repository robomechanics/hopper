import serial
import time

ser = serial.Serial('/dev/tty.usbmodem14101', 38400) # open serial port with Arduino

time.sleep(1)

while(True):
	serial_buffer = ser.readline()
	try:
		imu_update_whitespacechars = serial_buffer.split(',')
		imu_update_str = map(str.strip, imu_update_whitespacechars)
		imu_update = map(float, imu_update_str)
		[ax, ay, az, y, p, r] = imu_update
		print(imu_update)
	except:
		print("Invalid buffer response")
		print(serial_buffer)
