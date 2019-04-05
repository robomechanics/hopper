import serial
from matplotlib import pyplot as plt
import numpy as np

np.set_printoptions(suppress=True)

ser = serial.Serial('/dev/tty.usbmodem14101', 38400) # open serial port with Arduino

plt.axis([0,1000,0,2])
imu_data = np.empty((0, 6))
while(True):
	imu_update_whitespacechars = ser.readline().split(',')
	imu_update_str = map(str.strip, imu_update_whitespacechars)
	imu_update = np.array(map(float, imu_update_str))
	imu_data = np.append(imu_data, [imu_update], axis = 0)
	
# imu_data is imu_message_count x 6 array (where 6 is number of sensor updates per message)
# 	to access a specific value, imu_data[index, value_num] (ax,ay,az,y,p,r)