import numpy as np
from matplotlib import pyplot as plt
import time

class Plotter:
	def __init__(self, id):
		self.id = id
		self.initialize_canvas()

	def initialize_canvas(self):
		plt.ion()
		self.fig = plt.figure()
		self.ay = self.fig.add_subplot(211)
		self.dline, = plt.plot([0], [0])
		# ay.set_xlim([-1, 99])
		# ay.set_ylim([0,220])
		self.ax = self.fig.add_subplot(212)
		self.dlinef, = plt.plot([0], [0])
		plt.show(block=False)

		self.data1 = np.array([0])
		self.freq = np.array([0])

	def update_canvas(self, time, data, data2):
		self.dline.set_xdata(time)
		self.dline.set_ydata(data)
		self.ay.set_xlim([min(time), max(time)])
		self.ay.set_ylim([min(data), max(data)])

		# self.dlinef.set_xdata(self.freq)
		# self.dlinef.set_ydata(10 * np.log(self.data1))
		# self.ax.set_xlim([min(self.freq), max(self.freq)])
		# self.ax.set_ylim([-20, 20.0])

		self.dlinef.set_xdata(time)
		self.dlinef.set_ydata(data2)
		self.ax.set_xlim([min(time), max(time)])
		self.ax.set_ylim([min(data2), max(data2)])

		self.fig.canvas.draw()

	def update_canvasf(self, freq, data1):
		self.freq = freq
		self.data1 = data1
