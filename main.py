import serial

ser = serial.Serial('/dev/ttyUSB0', 115200) # open serial port with Arduino

while(true):
	imu_update = ser.readline()
	print(type(imu_update))
	print(imu_update)
