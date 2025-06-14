#!/usr/bin/env python3
from haptiquad_plot.libs import PlotterBase
from haptiquad_plot.libs import PlotContainer
import rclpy
from tkinter import ttk
import tkinter as tk
import numpy as np
from copy import deepcopy

from haptiquad_msgs.msg import ResidualsStamped, ObserverGains, ResidualErrorStamped

force_labels = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz']

class ResidualPlotter(PlotterBase):

	def __init__(self):

		PlotterBase.__init__(self, "ResidualPlotter")

		self.joint_names = []
		self.err_start_time = None

		self.r_int = {self.legs_prefix[0]: np.empty((3,0)), 
					self.legs_prefix[1]: np.empty((3,0)), 
					self.legs_prefix[2]: np.empty((3,0)), 
					self.legs_prefix[3]: np.empty((3,0))}
		
		self.joint_labels = {self.legs_prefix[0]: [], 
					self.legs_prefix[1]: [], 
					self.legs_prefix[2]: [], 
					self.legs_prefix[3]: []}
		
		self.time = np.empty(0)
		self.err_time = np.empty(0)


		self.r_ext = np.empty((6,0))

		self.err_int = {self.legs_prefix[0]: np.empty((3,0)), 
				  	self.legs_prefix[1]: np.empty((3,0)), 
					self.legs_prefix[2]: np.empty((3,0)), 
					self.legs_prefix[3]: np.empty((3,0))}
		self.err_ext = np.empty((6,0))

		self.plots['External'].autoscale = True


		self.residual_sub = self.create_subscription(ResidualsStamped, '/haptiquad_ros2/residuals', self.callback, 10)
		self.residual_error_sub = self.create_subscription(ResidualErrorStamped, '/haptiquad_ros2/residual_errors', self.errors_callback, 10)
		self.gains_pub = self.create_publisher(ObserverGains, '/haptiquad_ros2/gains', 10)
		self.gains_msg = ObserverGains()



	def set_gains(self):
		self.gains_msg.k_int.data = float(self.k_int.get())
		self.gains_msg.k_ext.data = float(self.k_ext.get())
		self.gains_pub.publish(self.gains_msg)


	def add_GUI(self):

		self.autoscroll_butt = ttk.Checkbutton(self, style='Toggle.TButton', text="Autoscroll", command=self.set_autoscroll)
		self.autoscroll_butt.grid(row=0, column=7, padx = 5, pady = 10, sticky="w")

		# self.scroll_range.set("2000")

		self.scroll_range_spin = ttk.Spinbox(self, from_=10.0, to=10000.0, increment=5.0, textvariable=self.scroll_range, width=5)
		self.scroll_range_spin.grid(row = 0, column=8, padx = (20,5), pady = 10, sticky="we")

		self.set_scroll_range_butt = ttk.Button(self, text="Set", command=self.set_scroll_range, width=2)
		self.set_scroll_range_butt.grid(row = 0, column=9, padx=(5, 10), sticky="we", pady=10)
		self.set_scroll_range_butt.config(state=tk.DISABLED)	

		ttk.Separator(self, orient=tk.VERTICAL).grid(row = 0, column=10, sticky="ns", pady = 5, padx = 5)

		self.k_int = tk.StringVar()
		self.k_int.set("0.3")

		self.k_ext = tk.StringVar()
		self.k_ext.set("0.3")

		ttk.Label(self, text="K int:").grid(row = 0, column=11, sticky="ns", pady = 5, padx = 5)

		self.k_int_spin = ttk.Spinbox(self, from_=0.0, to=100.0, increment=0.01, textvariable=self.k_int, width=4)
		self.k_int_spin.grid(row = 0, column=12, padx=(5, 10), sticky="we", pady=10)


		ttk.Label(self, text="K ext:").grid(row = 0, column=13, sticky="ns", pady = 5, padx = 5)

		self.k_ext_spin = ttk.Spinbox(self, from_=0.0, to=100.0, increment=0.01, textvariable=self.k_ext, width=4)
		self.k_ext_spin.grid(row = 0, column=14, padx=(5, 10), sticky="we", pady=10)

		self.send_gains_butt = ttk.Button(self, text="Set Gains", command=self.set_gains, width=2)
		self.send_gains_butt.grid(row = 0, column=15, padx=(5, 10), sticky="we", pady=10)

		
		self.show_err = tk.BooleanVar()
		self.show_err.set(False)
		self.last_mode = self.show_err.get()
		self.show_err_butt = ttk.Checkbutton(self, style='Toggle.TButton', text="Errors", variable=self.show_err)
		self.show_err_butt.grid(row=0, column=16, padx = (50, 20), pady=10, sticky="w")


		self.plots = [PlotContainer(self, f"Internal Residual - {self.legs_prefix[i]}") for i in range(4)]
		self.plots = {
			self.legs_prefix[0]:PlotContainer(self, f"Internal Residual - {self.legs_prefix[0]}"),
			self.legs_prefix[1]:PlotContainer(self, f"Internal Residual - {self.legs_prefix[1]}"),
			self.legs_prefix[2]:PlotContainer(self, f"Internal Residual - {self.legs_prefix[2]}"),
			self.legs_prefix[3]:PlotContainer(self, f"Internal Residual - {self.legs_prefix[3]}"),
			'External':PlotContainer(self, "External Residual"),
		}
		self.ext_plot = PlotContainer(self, "External Residual")

		self.plots[self.legs_prefix[0]].grid(row=1, column=0, padx=0, pady=0, 	columnspan=9,	sticky="w")
		self.plots[self.legs_prefix[1]].grid(row=2, column=0, padx=0, pady=0, 	columnspan=9,	sticky="w")
		self.plots[self.legs_prefix[2]].grid(row=1, column=9, padx=0, pady=0, 	columnspan=9,	sticky="w")
		self.plots[self.legs_prefix[3]].grid(row=2, column=9, padx=0, pady=0, 	columnspan=9,	sticky="w")
		self.plots['External'].grid(row=3, column=0, padx=0, pady=0, 	columnspan=18,	sticky="w")

		self.init_from_params()


	def on_resize(self, event):

			#Update to get the current values
			self.update()
			curr_size = (self.winfo_width(), self.winfo_height())

			if self.previous_size != curr_size:

				height = self.winfo_height() / 3 - 40
				width = self.winfo_width() / 2

				# last_plot_height = self.winfo_height() / 3 - 50
				last_plot_width = self.winfo_width() -50

				for i in range(4):

					self.plots[self.legs_prefix[i]].canvas.get_tk_widget().config(width=width, height=height)
					self.plots[self.legs_prefix[i]].fast_update()

				self.plots['External'].canvas.get_tk_widget().config(width=last_plot_width, height=height)
				self.plots['External'].fast_update()

			self.previous_size = curr_size

	def start_listening(self):		
		
		self.listening = not self.listening

		if self.listening:

			for plot in self.plots.values():

				plot.clear()
				plot.adjust_plots(0.1, 0.16, 0.938, 0.88, 0.2, 0.165)

			self.ext_plot.clear()

			self.r_int = {self.legs_prefix[0]: np.empty((3,0)), 
					self.legs_prefix[1]: np.empty((3,0)), 
					self.legs_prefix[2]: np.empty((3,0)), 
					self.legs_prefix[3]: np.empty((3,0))}
			self.r_ext = np.empty((6,0))
			self.err_int = {self.legs_prefix[0]: np.empty((3,0)), 
				  	self.legs_prefix[1]: np.empty((3,0)), 
					self.legs_prefix[2]: np.empty((3,0)), 
					self.legs_prefix[3]: np.empty((3,0))}
			self.err_ext = np.empty((6,0))
			self.time = np.empty(0)
			self.err_time = np.empty(0)



	def proces_mode_int(self, i):

		key = self.legs_prefix[i]
		labels = self.joint_labels[key]
		pause = self.pause.get()

		if self.show_err.get():

			to_plot = self.err_int[key] if not pause else self.frozen_data['err_int'][key]
			time = self.err_time if not pause else self.frozen_data['err_time']
			title = f"Internal Residual Error - {key}"

		else:

			to_plot = self.r_int[key] if not pause else self.frozen_data['int'][key]
			time = self.time if not pause else self.frozen_data['time']
			title = f"Internal Residual - {key}"

		return to_plot, time, labels, title
	

	def proces_mode_ext(self):

		pause = self.pause.get()
		if self.show_err.get():

			to_plot = self.err_ext if not pause else self.frozen_data['err_ext']
			time = self.err_time if not pause else self.frozen_data['err_time']
			title = "External Residual Error"

		else:

			to_plot = self.r_ext if not pause else self.frozen_data['ext']
			time = self.time if not pause else self.frozen_data['time']
			title = "External Residual"

		return to_plot, time, title


	def update_plots(self):

		if self.pause.get() and type(self.frozen_data) == type(None):
			
			self.frozen_data = {}
			self.frozen_data['int'] = deepcopy(self.r_int)
			self.frozen_data['ext'] = deepcopy(self.r_ext)
			self.frozen_data['err_int'] = deepcopy(self.err_int)
			self.frozen_data['err_ext'] = deepcopy(self.err_ext)
			self.frozen_data['time'] = deepcopy(self.time)
			self.frozen_data['err_time'] = deepcopy(self.err_time)


		elif not self.pause.get() and type(self.frozen_data) == dict:

			del self.frozen_data
			self.frozen_data = None


		if not self.last_mode == self.show_err.get():
			self.last_mode = self.show_err.get()
			for plt in self.plots.values():
				plt.clear()

		for i in range(4):

			to_plot, time, labels, title = self.proces_mode_int(i)

			if not to_plot.shape[1] == time.shape[0]:
				continue

			self.plots[self.legs_prefix[i]].update_plot(to_plot, labels,
												xlabel="Time [s]", ylabel="Momentum [Nm]", time = time,
												title=title)

		to_plot, time, title = self.proces_mode_ext()

	
		if not time.shape[0] == to_plot.shape[1]:
			return
		
		self.plots['External'].update_plot(to_plot, force_labels, xlabel="Time [s]", ylabel="Force [N]", time = time, title = title)



	def callback(self, msg):

		if not self.listening:
			return
		
		self.joint_names = msg.names		

		t = msg.stamp.sec + 1e-9 * msg.stamp.nanosec
		if self.start_time == None:
			self.start_time = t			
		
		normalized_time = t-self.start_time
		self.time = np.append(self.time, normalized_time)	
		
		for i in range(4):

			leg_values = np.array([msg.r_int[3*i+j] for j in range(3)]).reshape((3,1))

			leg_key = self.joint_names[3*i].split('_')[0]

			self.joint_labels[leg_key] = [self.joint_names[3*i+j] for j in range(3)]

			self.r_int[leg_key] = np.hstack((self.r_int[leg_key], leg_values))

			if self.r_int[leg_key].shape[1] > self.limit:
					self.r_int[leg_key] = self.r_int[leg_key][:, 1:]

		new_values = np.array([value for value in msg.r_ext]).reshape((6,1))
		self.r_ext = np.hstack((self.r_ext, new_values))

		if self.r_ext.shape[1] > self.limit:
				self.r_ext = self.r_ext[:, 1:]


		if self.time.shape[0] > self.limit:
				self.time = self.time[1:]



	def errors_callback(self, msg):

		if not self.listening:
			return
		
		self.joint_names = msg.names		
		
		for i in range(4):

			leg_values = np.array([msg.err_int[3*i+j] for j in range(3)]).reshape((3,1))

			leg_key = self.joint_names[3*i].split('_')[0]

			self.joint_labels[leg_key] = [self.joint_names[3*i+j] for j in range(3)]

			self.err_int[leg_key] = np.hstack((self.err_int[leg_key], leg_values))

			if self.err_int[leg_key].shape[1] > self.limit:
					self.err_int[leg_key] = self.err_int[leg_key][:, 1:]

		new_values = np.array([value for value in msg.err_ext]).reshape((6,1))
		self.err_ext = np.hstack((self.err_ext, new_values))

		t = msg.stamp.sec + 1e-9 * msg.stamp.nanosec
		if self.err_start_time == None:
			self.err_start_time = t			
		
		normalized_time = t-self.err_start_time
		self.err_time = np.append(self.err_time, normalized_time)	

		if self.err_ext.shape[1] > self.limit:
				self.err_ext = self.err_ext[:, 1:]

		if self.err_time.shape[0] > self.limit:
				self.err_time = self.err_time[1:]






def main(args=None):
	rclpy.init(args=args)
	plotter = ResidualPlotter()
	plotter.run()


if __name__ == '__main__':
	main()