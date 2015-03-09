import numpy as np
from matplotlib import pyplot as plt
import time
import kalman

def calculate_displacement_simple(ax, old_a, old_v, old_p, dt, index):
	vx = 0.0
	px = 0.0
	if abs(ax) < 30.0 or abs(ax - old_a) < 10: ax = 0
	if(index == 0):
		vx = 0
		px = 0 + vx * dt + 0.5 * ax * dt * dt
	else:
		# vx = old_v + ax * dt
		# vx = 0
		# if abs(ax) < 0.01 or abs(ax - old_a) < 0.001: vx = 0
		# kalman_filter_v.input_latest_noisy_measurement(vx)
		# vx = kalman_filter_v.get_latest_estimated_measurement()
		# px = old_p + old_v * dt + 0.5 * ax * dt * dt
		px = old_p + 0.5 * ax * dt * dt
		# kalman_filter_p.input_latest_noisy_measurement(px)
		# px = kalman_filter_p.get_latest_estimated_measurement()
	return [vx, px]

data = np.loadtxt('data_z.csv', delimiter=',')
time = data[2501:, 0] # skip bad data
acceleration = data[2501:, 1]
velocity = data[2501:, 2]
position = data[2501:, 3]

LSB_G_bad = 16384.0 / 2.0
LSB_G = 16384.0
k_acceleration = []
k_velocity = []
k_position = []
kalman_filter_a = kalman.KalmanFilter(50.0, np.std(acceleration) ** 2)
dt = time[1] - time[0]
for index in range(0, len(acceleration)):
	a = acceleration[index] - 200.0 # * LSB_G_bad / LSB_G
	if(abs(a) > 700): a = a / 10.0
	a_old = 0
	v = 0.0
	p = 0.0
	if(index > 0):
		v = k_velocity[index - 1]
		p = k_position[index - 1]
	kalman_filter_a.input_latest_noisy_measurement(a)
	new_a = kalman_filter_a.get_latest_estimated_measurement()
	k_acceleration.append(new_a)
	if(index > 0): a_old = k_acceleration[index - 1]
	[new_v, new_p] = calculate_displacement_simple(new_a, a_old, v, p, dt, index)
	k_velocity.append(new_v)
	k_position.append(new_p)

# fig = plt.figure()
# ay = fig.add_subplot(311)
# plt.plot(time, acceleration, 'b', time, k_acceleration, 'r')
# ay = fig.add_subplot(312)
# plt.plot(time, velocity, 'b', time, k_velocity, 'r')
# az = fig.add_subplot(313)
# plt.plot(time, position, 'b', time, k_position, 'r')
# plt.show()

fig = plt.figure()
ay = fig.add_subplot(311)
plt.plot(time, k_acceleration, 'r')
ay = fig.add_subplot(312)
plt.plot(time, k_velocity, 'r')
az = fig.add_subplot(313)
plt.plot(time, k_position, 'r')
plt.show()