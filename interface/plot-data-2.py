import numpy as np
from matplotlib import pyplot as plt
import time
import kalman_2 as kalman

def calculate_displacement_simple(ax, old_v, old_p, dt, index):
	vx = 0.0
	px = 0.0
	if(index == 1):
		vx = 0
		px = 0 + vx * dt + 0.5 * ax * dt * dt
	else:
		vx = old_v + ax * dt
		# kalman_filter_v.input_latest_noisy_measurement(vx)
		# vx = kalman_filter_v.get_latest_estimated_measurement()
		px = old_p + old_v * dt + 0.5 * ax * dt * dt
		# kalman_filter_p.input_latest_noisy_measurement(px)
		# px = kalman_filter_p.get_latest_estimated_measurement()
	return [vx, px]

data = np.loadtxt('data.csv', delimiter=',')
time = data[2501:, 0] # skip bad data
acceleration = data[2501:, 1]
velocity = data[2501:, 2]
position = data[2501:, 3]

k_acceleration = []
k_velocity = []
k_position = []
kalman_filter_a = kalman.KalmanFilter1D(x0=0, P=1e-3, R=np.std(acceleration) ** 2, Q=np.std(acceleration))
# kalman_filter_v = kalman.KalmanFilter(1.0, np.std(velocity) ** 2)
# kalman_filter_p = kalman.KalmanFilter(1.0e-6, np.std(position) ** 2)
dt = time[1] - time[0]
for index in range(0, len(acceleration)):
	a = acceleration[index]
	v = 0.0
	p = 0.0
	if(index > 0):
		v = velocity[index - 1]
		p = position[index - 1]
	kalman_filter_a.input_latest_noisy_measurement(a)
	new_a = kalman_filter_a.get_latest_estimated_measurement()
	[new_v, new_p] = calculate_displacement_simple(new_a, v, p, dt, index)
	k_acceleration.append(new_a)
	k_velocity.append(new_v)
	k_position.append(new_p)

fig = plt.figure()
ay = fig.add_subplot(311)
plt.plot(time, acceleration, 'b', time, k_acceleration, 'r')
ay = fig.add_subplot(312)
plt.plot(time, velocity, 'b', time, k_velocity, 'r')
az = fig.add_subplot(313)
plt.plot(time, position, 'b', time, k_position, 'r')
plt.show()