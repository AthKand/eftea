# -*- coding: utf-8 -*-
# @Author: ath
# @Date:   2023-07-17 14:35:16
# @Last Modified by:   ath
# @Last Modified time: 2023-07-26 11:20:33


'''

TO-DO: 
[] Test with mutliple parameters for KF
[] add different modes (rotation calibration, point of application)
[] argparser for CLI args and mode set up 

'''

import os
import time
import csv
import random
import numpy as np
import sys
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import matplotlib.animation as animation
from pynput import keyboard
from collections import deque
from scipy.optimize import minimize

# extras
from util.read_data import follow
from util.kf import KalmanFilter


def draw(axis, data, key):
	axis.clear()
	for i in range(1, len(data)):
		axis.plot(list(data[0]), data[i])

	axis.legend([f'{key}x', f'{key}y', f'{key}z'], loc = 1)

	if key == 'F':
		axis.set_title('Force (N)')
		axis.set_ylabel('Force (N)')
	elif key == 'M':
		axis.set_title('Moment (Nm)')
		axis.set_ylabel('Moment(Nm)')


class FORCE_TORQUE:

	def __init__(self):

		self.log = "/home/ath/work/driver/Linux/bin/log"
		self.csv = 'output/find_rot.csv'

		self.listener = keyboard.Listener(on_press=self._on_press)

		# Rotation and Translation matrices to find pos on mTMS head
		self.R = np.load('resources/Rot.npy')
		self.T = np.load('resources/Tr.npy')

		# Capture new reference?
		self.REF_FLAG = False

		# reference Force-Torque values 
		self.f_ref = np.array([0.0, 0.0, 0.0])
		self.m_ref = np.array([0.0, 0.0, 0.0])

		# self.kf_r = [KalmanFilter(initial_state=0, initial_uncertainty=1, measurement_uncertainty=10, process_noise=1) for _ in range(3)]
		# self.kf_r_tran = [KalmanFilter(initial_state=0, initial_uncertainty=30, measurement_uncertainty=100, process_noise=10) for _ in range(3)]

		# Set tool origin and size bounds
		# ..... based on size of mTMS
		self.orig = np.array([0.0, 0.0, 0.05])
		self.bounds_x = (-0.15, 0.15)        
		self.bounds_y = (-0.15, 0.15)
		self.bounds_z = (0, 0.05)

		# for smoothing
		self.F_values = deque(maxlen=6)
		self.M_values = deque(maxlen=6)
		self.r_values = deque(maxlen=20)

		# to record values for rotation calculation
		self.recording = False
		self.strt = 0
		self.rot = []

		# create plots for Force
		self.fig = plt.figure()
		self.ax = self.fig.add_subplot(2, 1, 1)
		self.x_f = np.linspace(0, 100, 100)
		self.y_fx, self.y_fy, self.y_fz   = deque([0] * 100, maxlen = 100), deque([0] * 100, maxlen = 100), deque([0] * 100, maxlen = 100)

		self.ax2 = self.fig.add_subplot(2, 1, 2)
		self.x_m = np.linspace(0, 100, 100)
		self.y_mx, self.y_my, self.y_mz   = deque([0] * 100, maxlen = 100), deque([0] * 100, maxlen = 100), deque([0] * 100, maxlen = 100)

		# create plots for point of application
		self.fig3 = plt.figure()
		self.ax3 = self.fig3.add_subplot(1, 1, 1)
		self.square = FancyBboxPatch((-12, -12), 24, 24, alpha=0.5, boxstyle="round,pad=3")
		self.ax3.add_patch(self.square)
		self.ax3.set_xlim(-20, 20)
		self.ax3.set_ylim(-20, 20)
		self.point, = self.ax3.plot(0, 0, 'ro', markersize=10)
		# self.kf_point, = self.ax2.plot(0, 0, 'g+', markersize=20)

		self.listener.start()

		ani = animation.FuncAnimation(self.fig, self.loop, fargs=(), interval=1)
		plt.show()

		#self.loop()

	def _on_press(self, key):
		'''
		Listen to keyboard press while the loop to display 
		F - T values is running. 

		Returns: None - regular actions
				 False - stop listener

		'''
		if key == keyboard.Key.esc:
			return False  # stop listener
		try:
			k = key.char  # single-char keys
		except:
			k = key.name  # other keys
		if k == 'n':
			print('Normalising . . . .')
			self.REF_FLAG = True		


	def _func(self, r, F, M, orig):
		'''
		Function to minimise objective function

		Returns: Norm of r x F - M (Np Array)  
		'''
		# Check if r is outside the box
		return np.linalg.norm(np.cross(r - orig, F) - M)

	def find_r(self, F, M):
		'''
		Find point of application of force

		Returns: Minimised point of application (Np Array) 
		'''

		# initial guess
		r0 = np.array([0.0, 0.0, 0.05])

		# find r that minimizes the objective function
		res = minimize(self._func, r0, 
					   args=(F, M, self.orig), 
					   method='Nelder-Mead', 
					   bounds=(self.bounds_x, self.bounds_y, self.bounds_z))
		
		r_min = res.x * 100  # multiply by 100 to get value in cm

		return [round(r_min[i], 1) for i in range(0, len(r_min))]


	def loop(self, i):
		'''
		Loop to read live sensor data and perform relevant operations. 

		Performs: Live plotting, 
				write-to-csv, 
				normalise(tare) values
		'''
		with open(self.log, 'rb') as f:
			try:  # catch OSError in case of a one line file 
				f.seek(-2, os.SEEK_END)
				while f.read(1) != b'\n':
					f.seek(-2, os.SEEK_CUR)
			except OSError:
				f.seek(0)
			l = f.readline().decode()

		data = l[l.find("( ")+1:l.find(" )")]
		F_vect  = np.array(data.split(" , ")[:3], dtype = 'float64')
		M_vect = np.array(data.split(" , ")[3:], dtype = 'float64')
		F_normalised = F_vect - self.f_ref
		M_normalised = M_vect - self.m_ref

		# Add new value to the lists
		self.F_values.append(F_normalised)
		self.M_values.append(M_normalised)

		# Compute the average of the last N values
		F_avg = np.mean(self.F_values, axis=0)
		M_avg = np.mean(self.M_values, axis=0)

		for num, item in enumerate([self.y_fx, self.y_fy, self.y_fz]):
			item.append(F_normalised[num])

		for num, item in enumerate([self.y_mx, self.y_my, self.y_mz]):
			item.append(M_normalised[num])

		flist = [self.x_f, self.y_fx, self.y_fy, self.y_fz]
		mlist = [self.x_m, self.y_mx, self.y_my, self.y_mz]
		draw(self.ax, data = flist, key = 'F')
		draw(self.ax2, data = mlist, key = 'M')

		# Format plot
		plt.xticks(rotation=45, ha='right')
		plt.subplots_adjust(bottom=0.30)

		r = self.find_r(F_avg, M_avg)
		# rotating R to the correct coordinates
		r_tran = self.R @ r + self.T
		r_tran = [round(r_tran[i], 1) for i in range(0, len(r_tran))]

		if not (-15 <= r_tran[0] <= 15 and 
				-15 <= r_tran[1] <= 15):
			self.point.set_data([0], [0])

		else:
			if F_avg[2] < -1: 
				self.point.set_data(r_tran[0], r_tran[1])
			else:
				self.point.set_data([0], [0])

		# Redraw the plot
		self.fig3.canvas.draw()

		# Taring the values
		# activated on key press ('n')
		if self.REF_FLAG:
			self.f_ref = F_vect
			self.m_ref = M_vect
			self.REF_FLAG = False



if __name__ == '__main__':
	FORCE_TORQUE()

