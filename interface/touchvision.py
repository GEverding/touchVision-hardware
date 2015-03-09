from touchvision_kinematics import *
from random import randint
import serial

ser = serial.Serial('/dev/tty.usbmodem1411', 115200) # Confirm the baud rate
kinematics = KinematicsSolver(1)

while(1):
	incoming_data = ser.readline()
	# print(incoming_data)
	if("DMP ready! Waiting for first interrupt..." in incoming_data): break

while(1):
	incoming_data = ser.readline()
	packet = incoming_data.split(constants.PACKET_DELIMINATOR)
	for sub_packet in packet:
		# print(sub_packet)
		data = sub_packet.split(constants.SUB_PACKET_DELIMINATOR)
		kinematics.process_acceleration_sample([float(data[0]), float(data[1]), float(data[2])], float(data[3]), float(data[4]))
		[t, x, y, z, pressure] = kinematics.get_latest_measurements()