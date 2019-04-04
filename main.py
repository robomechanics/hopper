import serial

ser = serial.Serial('/dev/tty.usbmodem14101', 38400) # open serial port with Arduino

while(True):
	imu_update_whitespacechars = ser.readline().split(',')
	imu_update_str = map(str.strip, imu_update_whitespacechars)
	imu_update = map(float, imu_update_str)
	[ax, ay, az, y, p, r] = imu_update
	print(imu_update)
