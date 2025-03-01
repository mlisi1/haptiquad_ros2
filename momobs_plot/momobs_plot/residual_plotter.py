#!/usr/bin/env python3
from momobs_plot.libs import PlotterBase
from momobs_plot.libs import PlotContainer
import rclpy
from tkinter import ttk
import tkinter as tk
import numpy as np

from momobs_msgs.msg import ResidualsStamped, ObserverGains, ResidualErrorStamped

legs = ["LF", "RF", "LH", "RH"]
label = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz']

class ResidualPlotter(PlotterBase):

	def __init__(self):

		PlotterBase.__init__(self, "ResidualPlotter")

		self.joint_names = []

		self.r_int = {legs[0]: np.empty((3,0)), 
					legs[1]: np.empty((3,0)), 
					legs[2]: np.empty((3,0)), 
					legs[3]: np.empty((3,0))}
		
		self.joint_labels = {legs[0]: [], 
					legs[1]: [], 
					legs[2]: [], 
					legs[3]: []}


		self.r_ext = np.empty((6,0))

		self.err_int = {legs[0]: np.empty((3,0)), 
				  	legs[1]: np.empty((3,0)), 
					legs[2]: np.empty((3,0)), 
					legs[3]: np.empty((3,0))}
		self.err_ext = np.empty((6,0))

		self.plots['External'].autoscale = True


		self.residual_sub = self.create_subscription(ResidualsStamped, '/momobs_ros2/residuals', self.callback, 10)
		self.residual_error_sub = self.create_subscription(ResidualErrorStamped, '/momobs_ros2/residual_errors', self.errors_callback, 10)
		self.gains_pub = self.create_publisher(ObserverGains, '/momobs_ros2/gains', 10)
		self.gains_msg = ObserverGains()



	def set_gains(self):
		self.gains_msg.k_int.data = float(self.k_int.get())
		self.gains_msg.k_ext.data = float(self.k_ext.get())
		self.gains_pub.publish(self.gains_msg)


	def add_GUI(self):

		self.autoscroll_butt = ttk.Checkbutton(self, style='Toggle.TButton', text="Autoscroll", command=self.set_autoscroll)
		self.autoscroll_butt.grid(row=0, column=6, padx = 5, pady = 10, sticky="w")

		self.scroll_range = tk.StringVar()
		self.scroll_range.set("2000")

		self.scroll_range_spin = ttk.Spinbox(self, from_=10.0, to=10000.0, increment=5.0, textvariable=self.scroll_range, width=5)
		self.scroll_range_spin.grid(row = 0, column=7, padx = (20,5), pady = 10, sticky="we")

		self.set_scroll_range_butt = ttk.Button(self, text="Set", command=self.set_scroll_range, width=2)
		self.set_scroll_range_butt.grid(row = 0, column=8, padx=(5, 10), sticky="we", pady=10)
		self.set_scroll_range_butt.config(state=tk.DISABLED)	

		ttk.Separator(self, orient=tk.VERTICAL).grid(row = 0, column=9, sticky="ns", pady = 5, padx = 5)

		self.k_int = tk.StringVar()
		self.k_int.set("0.3")

		self.k_ext = tk.StringVar()
		self.k_ext.set("0.3")

		ttk.Label(self, text="K int:").grid(row = 0, column=10, sticky="ns", pady = 5, padx = 5)

		self.k_int_spin = ttk.Spinbox(self, from_=0.0, to=100.0, increment=0.01, textvariable=self.k_int, width=4)
		self.k_int_spin.grid(row = 0, column=11, padx=(5, 10), sticky="we", pady=10)


		ttk.Label(self, text="K ext:").grid(row = 0, column=12, sticky="ns", pady = 5, padx = 5)

		self.k_ext_spin = ttk.Spinbox(self, from_=0.0, to=100.0, increment=0.01, textvariable=self.k_ext, width=4)
		self.k_ext_spin.grid(row = 0, column=13, padx=(5, 10), sticky="we", pady=10)

		self.send_gains_butt = ttk.Button(self, text="Set Gains", command=self.set_gains, width=2)
		self.send_gains_butt.grid(row = 0, column=14, padx=(5, 10), sticky="we", pady=10)

		self.show_err = tk.BooleanVar()
		self.show_err.set(False)
		self.show_err_butt = ttk.Checkbutton(self, style='Toggle.TButton', text="Errors", variable=self.show_err)
		self.show_err_butt.grid(row=0, column=15, padx = (50, 20), pady=10, sticky="w")


		self.plots = [PlotContainer(self, f"Internal Residual - {legs[i]}") for i in range(4)]
		self.plots = {
			legs[0]:PlotContainer(self, f"Internal Residual - {legs[0]}"),
			legs[1]:PlotContainer(self, f"Internal Residual - {legs[1]}"),
			legs[2]:PlotContainer(self, f"Internal Residual - {legs[2]}"),
			legs[3]:PlotContainer(self, f"Internal Residual - {legs[3]}"),
			'External':PlotContainer(self, "External Residual"),
		}
		self.ext_plot = PlotContainer(self, "External Residual")

		self.plots[legs[0]].grid(row=1, column=0, padx=0, pady=0, 	columnspan=8,	sticky="w")
		self.plots[legs[1]].grid(row=2, column=0, padx=0, pady=0, 	columnspan=8,	sticky="w")
		self.plots[legs[2]].grid(row=1, column=8, padx=0, pady=0, 	columnspan=8,	sticky="w")
		self.plots[legs[3]].grid(row=2, column=8, padx=0, pady=0, 	columnspan=8,	sticky="w")
		self.plots['External'].grid(row=3, column=0, padx=0, pady=0, 	columnspan=16,	sticky="w")

		self.init_from_params()


	def on_resize(self, event):

			#Update to get the current values
			self.update()
			curr_size = (self.winfo_width(), self.winfo_height())

			if self.previous_size != curr_size:

				height = self.winfo_height() / 3 - 30
				width = self.winfo_width() / 2 -50

				# last_plot_height = self.winfo_height() / 3 - 50
				last_plot_width = self.winfo_width() -50

				for i in range(4):

					self.plots[legs[i]].canvas.get_tk_widget().config(width=width, height=height)
					self.plots[legs[i]].fast_update()

				self.plots['External'].canvas.get_tk_widget().config(width=last_plot_width, height=height)
				self.plots['External'].fast_update()

			self.previous_size = curr_size

	def start_listening(self):		
		
		self.listening = not self.listening

		if self.listening:

			for plot in self.plots.values():

				plot.clear()

			self.ext_plot.clear()

			self.r_int = {legs[0]: np.empty((3,0)), 
					legs[1]: np.empty((3,0)), 
					legs[2]: np.empty((3,0)), 
					legs[3]: np.empty((3,0))}
			self.r_ext = np.empty((6,0))
			self.err_int = {legs[0]: np.empty((3,0)), 
				  	legs[1]: np.empty((3,0)), 
					legs[2]: np.empty((3,0)), 
					legs[3]: np.empty((3,0))}
			self.err_ext = np.empty((6,0))

	def update_plots(self):

		for i in range(4):

			if self.show_err.get():

				self.plots[legs[i]].update_plot(self.err_int[legs[i]], self.joint_labels[legs[i]])

			else:

				self.plots[legs[i]].update_plot(self.r_int[legs[i]], self.joint_labels[legs[i]])


		if self.show_err.get():

			self.plots['External'].update_plot(self.err_ext, label)

		else:
			
			self.plots['External'].update_plot(self.r_ext, label)

		self.need_to_update = False

	def callback(self, msg):

		if not self.listening:
			return
		
		self.joint_names = msg.names		
		
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

		self.need_to_update = True


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

		if self.err_ext.shape[1] > self.limit:
				self.err_ext = self.err_ext[:, 1:]

		self.need_to_update = True





def main(args=None):
	rclpy.init(args=args)
	plotter = ResidualPlotter()
	plotter.run()


if __name__ == '__main__':
	main()