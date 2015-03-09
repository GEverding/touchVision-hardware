import constants
# import plotter
# import numpy as np
import kalman

class KinematicsSolver:

	def __init__(self, id=1):
		self.id = id
		self.define_arrays()
		self.define_logger()

	def define_arrays(self):
		# Sample counter
		self.sample_count = 0

		# Axes
		self.axes = [0, 1, 2] # x, y, z

		# Time interval
		self.dt = 0.02272727273

		self.time = 0

		# Acceleration arrays
		accel_x = [0.0, 0.0]
		accel_y = [0.0, 0.0]
		accel_z = [0.0, 0.0]
		self.accel = [accel_x, accel_z, accel_y]

		# Velocity arrays
		vel_x = [0.0, 0.0]
		vel_y = [0.0, 0.0] 
		vel_z = [0.0, 0.0]
		self.vel = [vel_x, vel_y, vel_z]

		# Position arrays
		pos_x = [0.0, 0.0]
		pos_y = [0.0, 0.0]
		pos_z = [0.0, 0.0]
		self.pos = [pos_x, pos_y, pos_z]

		# Pressure value
		self.normalized_pressure_value = 0.0

		# Dynamic acceleration
		self.accel_dyn = [ [], [], [] ]

		# Compensation offset
		self.offset = [0, 0, 200]
		self.accel_threshold = [4.0 * 0.05, 4.0 * 0.05, 4.0 * 0.05]
		self.delta_threshold = [4.0 * 0.005, 4.0 * 0.005, 4.0 * 0.005]

		kalman_filter_x = kalman.KalmanFilter(50.0, 1)
		kalman_filter_y = kalman.KalmanFilter(50.0, 1)
		kalman_filter_z = kalman.KalmanFilter(50.0, 1)
		self.kalman_filter = [kalman_filter_x, kalman_filter_y, kalman_filter_z]

	def define_logger(self):
		return 0
		# self.plotter = plotter.Plotter(self.id)
		# self.logged_time = []
		# self.logged_data = []
		# self.logged_data2 = []
		# fh = open("data_x.csv","w")
		# fh.close()
		# fh = open("data_y.csv","w")
		# fh.close()
		# fh = open("data_z.csv","w")
		# fh.close()

	def log_data(self, time):
		return 0
		# if(len(self.logged_time) > 2000):
		# 	self.logged_time = []
		# 	self.logged_data = []
		# 	self.logged_data2 = []
		# self.logged_time.append(time)
		# self.logged_data.append(self.pos[0][-1])
		# self.logged_data2.append(self.accel[0][-1])
		# self.plotter.update_canvas(self.logged_time, self.logged_data, self.logged_data2)
		# print("a: t = " + str(time) + " --> (" + str(self.accel[0][-1]) + ", " + str(self.accel[1][-1]) + ", " + str(self.accel[2][-1]) + ")")
		# print("p: t = " + str(time) + " --> (" + str(self.pos[0][-1]) + ", " + str(self.pos[1][-1]) + ", " + str(self.pos[2][-1]) + ")")
		# fh = open("data_x.csv", "a")
		# fh.write(str(time) + "," + str(self.accel[0][-1]) + "," + str(self.vel[0][-1]) + "," + str(self.pos[0][-1]) + "\n")
		# fh.close
		# fh = open("data_y.csv", "a")
		# fh.write(str(time) + "," + str(self.accel[1][-1]) + "," + str(self.vel[1][-1]) + "," + str(self.pos[1][-1]) + "\n")
		# fh.close
		# fh = open("data_z.csv", "a")
		# fh.write(str(time) + "," + str(self.accel[2][-1]) + "," + str(self.vel[2][-1]) + "," + str(self.pos[2][-1]) + "\n")
		# fh.close

	def process_acceleration_sample(self, raw_accel, raw_pressure, time):
		self.time = time
		self.sample_count = self.sample_count + 1
		for axis in self.axes:
			# self.adjust_dc_filter(raw_accel, axis)
			self.add_acceleration_data(raw_accel, time, axis)
			self.add_position_data(time, axis)
		self.log_data(time)
		self.add_pressure_data(raw_pressure)

	def get_latest_measurements(self):
		return [self.time, self.pos[0][-1], self.pos[1][-1], self.pos[2][-1], self.normalized_pressure_value]

	def reset(self):
		# Reset all position measurements
		# Leave Kalman filters running
		self.pos[0][0] = 0.0
		self.pos[0][1] = 0.0
		self.pos[1][0] = 0.0
		self.pos[1][1] = 0.0
		self.pos[2][0] = 0.0
		self.pos[2][1] = 0.0

	def adjust_dc_filter(self, raw_accel, axis):
		if(self.number_of_particles_filtered[axis] < constants.WINDOW_SIZE):
			# Add new particle to fill up the window (only happens when filter is starting up)
			self.particles_filtered[axis][self.sample_count - 1] = raw_accel[axis]
			self.moving_average[axis] = ((self.moving_average[axis] * self.number_of_particles_filtered[axis]) + raw_accel[axis]) / (self.number_of_particles_filtered[axis] + 1.0)
			self.number_of_particles_filtered[axis] = self.number_of_particles_filtered[axis] + 1
		else:
			# Add particle to existing full window
			oldest_particle = self.particles_filtered[axis][0]
			self.push_fifo(self.particles_filtered[axis], raw_accel[axis])
			self.moving_average[axis] = ((self.moving_average[axis] * constants.WINDOW_SIZE) - oldest_particle + raw_accel[axis]) / constants.WINDOW_SIZE
			self.number_of_particles_filtered[axis] = self.number_of_particles_filtered[axis] + 1

	def add_acceleration_data(self, raw_accel, time, axis):
		if(time > constants.SEND_POSITION_THRESHOLD):
			# ---Kalman Filter:
		    self.kalman_filter[axis].input_latest_noisy_measurement(raw_accel[axis])
		    filtered_particle = self.kalman_filter[axis].get_latest_estimated_measurement()
		    if(self.sample_count == 1):
			    self.accel[axis][1] = filtered_particle
		    else:
			    self.accel[axis][0] = self.accel[axis][1]
			    self.accel[axis][1] = filtered_particle

	def add_position_data(self, time, axis):
		if(time > constants.SEND_POSITION_THRESHOLD):
			# Don't start process position data until the time threshold has passed (the sensors go ballistic during start-up)
			self.calculate_displacement_fast(axis)

	def add_pressure_data(self, pressure_value):
		self.normalized_pressure_value = pressure_value / constants.PRESSURE_NORMALIZER

	def calculate_displacement_fast(self, axis):
		accel_last_index = self.get_FIFO_last_index()
		ax_old = self.accel[axis][0] * constants.ACCEL_G / constants.LSB_G
		ax = self.accel[axis][1] * constants.ACCEL_G / constants.LSB_G
		if ax_old > 700: ax_old = ax_old / 10.0
		if ax > 700: ax = ax / 10.0
		vx = 0.0
		px = 0.0
		if abs(ax) < self.accel_threshold[axis] or abs(ax - ax_old) < self.delta_threshold[axis]: ax = 0
		if(self.sample_count == 1):
			vx = 0
			px = self.dt + 0.5 * ax * self.dt * self.dt
		else:
			vx = 0
			px = self.pos[axis][0] + 0.5 * ax * self.dt * self.dt * 1.0 # magic factor
		# self.add_new_value_to_FIFO(self.vel[axis], vx, vel_last_index)
		# self.add_new_value_to_FIFO(self.pos[axis], px, pos_last_index)
		if(self.sample_count == 1):
			self.vel[axis][1] = vx
			self.pos[axis][1] = px
		else:
			self.vel[axis][0] = self.vel[axis][1]
			self.vel[axis][1] = vx
			self.pos[axis][0] = self.pos[axis][1]
			self.pos[axis][1] = px

	def calculate_displacement(self, axis):
		last_index = self.get_FIFO_last_index()
		ax = self.accel[axis][last_index] * constants.ACCEL_G / constants.LSB_G
		vx = 0
		px = 0
		trigger_sum = 0
		trigger_length = 4
		if(self.sample_count == 1):
			if(abs(ax) < constants.KILL_THRESHOLD_LOW or abs(ax) > constants.KILL_THRESHOLD_HIGH):
				# Kill ridiculous values
				ax = 0.0
				self.accel[axis][last_index] = 0
			vx = 0
			px = 0 + vx * self.dt + 0.5 * ax * self.dt * self.dt
		else:
			if(abs(ax) < constants.KILL_THRESHOLD_LOW or abs(ax) > constants.KILL_THRESHOLD_HIGH):
				# Kill ridiculous values
				ax = 0.0
				self.accel[axis][last_index] = 0
			# Trigger on a string of constant 0 values, ignore sudden jumps after a long set of zeros and stop moving (velocity to zero) 
			# This is a form of smoothing to ignore noise spikes and to make sure we get realistic velocity even if we aren't sampling enough
			trigger_sum = 0
			for a_i in range(len(self.accel[axis]) - 1, len(self.accel[axis]) - 1 - trigger_length):
				trigger_sum = trigger_sum + abs(self.accel[axis][a_i]) * constants.ACCEL_G / constants.LSB_G
			vx = self.vel[axis][last_index - 1] + ax * self.dt
			# Trigger sum only valid when the FIFO is full, but who cares (I can fix this, but I'm too lazy and the initial values are usually ignore anyway)
			if(trigger_sum == 0): vx = 0
			px = self.pos[axis][last_index - 1] + vx * self.dt + 0.5 * ax * self.dt * self.dt
		if(self.sample_count < constants.SAMPLE_MEMORY):
			# FIFO isn't full yet...
			self.vel[axis][last_index] = vx
			self.pos[axis][last_index] = px
		else:
			self.push_fifo(self.vel[axis], vx)
			self.push_fifo(self.pos[axis], px)

	def calculate_displacement_simple_threshold(self, axis):
		accel_last_index = self.get_FIFO_last_index()
		vel_last_index = accel_last_index - 1 # Velocity hasn't been added to the FIFO yet
		pos_last_index = accel_last_index - 1 # Position hasn't been added to the FIFO yet
		ax = self.accel[axis][accel_last_index] * constants.ACCEL_G / constants.LSB_G
		vx = 0
		px = 0
		if(self.sample_count == 1):
			vx = 0
			px = 0 + vx * self.dt + 0.5 * ax * self.dt * self.dt
		else:
			vx = self.vel[axis][vel_last_index] + ax * self.dt
			trigger_sum = 0
			trigger_length = 4
			for a_i in range(len(self.accel[axis]) - 1, len(self.accel[axis]) - 1 - trigger_length):
				if abs(self.accel[axis][a_i]) * constants.ACCEL_G / constants.LSB_G < constants.KILL_THRESHOLD_LOW:
					trigger_sum = trigger_sum + 0
				else: trigger_sum = trigger_sum + 1
			if(trigger_sum == 0): vx = 0
			px = self.pos[axis][pos_last_index] + vx * self.dt + 0.5 * ax * self.dt * self.dt
			print(px)
			print(vx)
			print(ax)
		self.add_new_value_to_FIFO(self.vel[axis], vx, vel_last_index)
		self.add_new_value_to_FIFO(self.pos[axis], px, pos_last_index)

	def calculate_displacement_simple(self, axis):
		accel_last_index = self.get_FIFO_last_index()
		vel_last_index = accel_last_index - 1 # Velocity hasn't been added to the FIFO yet
		pos_last_index = accel_last_index - 1 # Position hasn't been added to the FIFO yet
		ax = self.accel[axis][accel_last_index] * constants.ACCEL_G / constants.LSB_G
		vx = 0
		px = 0
		if(self.sample_count == 1):
			vx = 0
			px = 0 + vx * self.dt + 0.5 * ax * self.dt * self.dt
		else:
			vx = self.vel[axis][vel_last_index] + ax * self.dt
			px = self.pos[axis][pos_last_index] + self.vel[axis][vel_last_index] * self.dt + 0.5 * ax * self.dt * self.dt
		self.add_new_value_to_FIFO(self.vel[axis], vx, vel_last_index)
		self.add_new_value_to_FIFO(self.pos[axis], px, pos_last_index)

	def calculate_displacement_fft(self, axis):
		accel_last_index = self.get_FIFO_last_index()
		vel_last_index = accel_last_index - 1 # Velocity hasn't been added to the FIFO yet
		pos_last_index = accel_last_index - 1 # Position hasn't been added to the FIFO yet
		# ax = self.accel[accel_last_index][-1] * constants.ACCEL_G / constants.LSB_G
		self.update_accel_dynamic(axis, accel_last_index)
		accel_dyn_mean_subtracted = (np.array(self.accel_dyn[axis]) - self.mean(self.accel_dyn[axis])) * constants.ACCEL_G / constants.LSB_G

		# Calcualte fft
		accel_dyn_mean_subracted_fft = np.fft.fft(accel_dyn_mean_subtracted)
		freq = np.fft.fftfreq(accel_dyn_mean_subtracted.shape[-1])

		displacement_fft = [] # Set DC = 0 to avoid infinities
		displacement_fft_raw = []
		k2 = 2 * np.pi * 2 * np.pi
		for i in range(0, accel_dyn_mean_subracted_fft.size):
			if abs(freq[i]) > 0:
			    d_i = -accel_dyn_mean_subtracted[i] / ( freq[i] * freq[i] * (len(freq) / 2.0) * (len(freq) / 2.0)) # (k2 * freq[i] * freq[i])
			    displacement_fft_raw.append(d_i)
			    # if(abs(freq[i]) > 2 * np.pi * 100.0): d_i = d_i / 1000.0
			    # if(abs(freq[i]) < 2 * np.pi * 10.0): d_i = d_i / 1000.0
			    displacement_fft.append(d_i)
			else:
				displacement_fft_raw.append(0)
				displacement_fft.append(0)
		self.plotter.update_canvasf(freq * (len(freq) / 2.0) / (2.0 * np.pi), np.array(accel_dyn_mean_subracted_fft))

		# Inverse fft
		displacement_dyn = np.fft.ifft(np.array(displacement_fft))
		position = np.real(displacement_dyn[-1])
		if(self.sample_count > 1): position = self.pos[axis][pos_last_index] + position # If not the first sample, add the previous value to get correct position
		self.add_new_value_to_FIFO(self.pos[axis], position, pos_last_index)

	def update_accel_dynamic(self, axis, accel_last_index):
		if(len(self.accel_dyn[axis]) < constants.DYNAMIC_LENGTH):
			# Array is not full yet
			self.accel_dyn[axis].append(self.accel[axis][accel_last_index])
		else:
			self.accel_dyn[axis][0] = (self.accel_dyn[axis][0] + self.accel_dyn[axis][1]) / 2.0 # First entry is the mean of itself and second entry
			for i in range(1, len(self.accel_dyn[axis]) - 1):
				# Shift entries up
				self.accel_dyn[axis][i] = self.accel_dyn[axis][i + 1]
			self.accel_dyn[axis][-1] = self.accel[axis][accel_last_index]

	def mean(self, array):
		sum = 0
		for l in array:
			sum = sum + l
		return sum / float(len(array))

	def add_new_value_to_FIFO(self, fifo, value, last_index):
		if(self.sample_count < constants.SAMPLE_MEMORY):
			# FIFO isn't full yet...
			fifo[last_index + 1] = value
		else:
			self.push_fifo(fifo, value)

	def get_FIFO_last_index(self):
		last_index = -1 # Last index of the FIFO
		if(self.sample_count < constants.SAMPLE_MEMORY):
			# The FIFO size is not full yet. There are only sample_count values in the FIFO, so index that last value.
			last_index = self.sample_count - 1
		return last_index

	def push_fifo(self, fifo, new_value):
		for i in range(len(fifo) - 1):
			fifo[i] = fifo[i + 1]
		fifo[-1] = new_value
	




