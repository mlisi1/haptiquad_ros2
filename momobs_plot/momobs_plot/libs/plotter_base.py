from ament_index_python.packages import get_package_share_directory
from momobs_plot.libs.plot_libs import PlotContainer
import tkinter as tk
from tkinter import ttk
from rclpy.node import Node
import rclpy
from threading import Thread



class PlotterBase(Node, tk.Tk):


	def __init__(self, node_name):

		self.listening = False
		self.need_to_update = False
		self.autoscale = False
		self.autoscroll = True

		Node.__init__(self, node_name)
		tk.Tk.__init__(self)

		self.protocol('WM_DELETE_WINDOW', self.on_destroy)
		self.geometry("1920x1080")
		tk.Tk.wm_title(self, node_name)



		theme_path = f"{get_package_share_directory('momobs_plot')}/theme/azure.tcl"

		self.tk.call("source", theme_path)
		self.tk.call("set_theme", "light")
		self.previous_size = (0, 0)

		self.declare_parameter('x_lim', 1000)
		self.limit = self.get_parameter('x_lim').get_parameter_value().integer_value

		self.listen = ttk.Checkbutton(self, style='Toggle.TButton', text="Listen Topic", command=self.start_listening)
		self.listen.grid(row=0, column=0, padx = (50, 5), pady=10, sticky="w")

		ttk.Separator(self, orient=tk.VERTICAL).grid(row = 0, column=1, sticky="ns", pady = 5, padx = 5)

		self.autoscale_butt = ttk.Checkbutton(self, style='Toggle.TButton', text="Autoscale", command=self.set_autoscale)
		self.autoscale_butt.grid(row=0, column=2, padx = 5, pady = 10, sticky="w")

		self.scale_range = tk.StringVar()
		self.scale_range.set("5")

		self.scale_range_spin = ttk.Spinbox(self, from_=0.01, to=100.0, increment=0.01, textvariable=self.scale_range, width=5)
		self.scale_range_spin.grid(row = 0, column=3, padx = (20,5), pady = 10, sticky="we")

		self.set_range_butt = ttk.Button(self, text="Set", command=self.set_range, width=2)
		self.set_range_butt.grid(row = 0, column=4, padx=(5, 10), sticky="we", pady=10)

		ttk.Separator(self, orient=tk.VERTICAL).grid(row = 0, column=5, sticky="ns", pady = 5, padx = 5)

		self.plots = []

		self.spin_thread = Thread(target=rclpy.spin, args=(self,), daemon=True)
		self.spin_thread.start()

		self.add_GUI()

		self.bind("<Configure>", self.on_resize) 
		self.on_resize(None)




	def set_scroll_range(self):

		value = float(self.scroll_range.get())
		for plot in self.plots.values():
			plot.x_range = value

	def set_autoscroll(self):

		self.autoscroll = not self.autoscroll
		for plot in self.plots.values():
			plot.autoscroll_x = self.autoscroll
			plot.redraw_limits = True


		if self.autoscroll:

			self.set_scroll_range_butt.config(state=tk.DISABLED)

		else:

			self.set_scroll_range_butt.config(state=tk.NORMAL)


	def set_range(self):

		value = float(self.scale_range.get())
		for plot in self.plots.values():
			plot.range = (-value, value)

	def set_autoscale(self):

		self.autoscale = not self.autoscale

		for plot in self.plots.values():

			plot.autoscale = self.autoscale
			plot.redraw_limits = True

		if self.autoscale:
			self.set_range_butt.config(state=tk.DISABLED)
		else:
			self.set_range_butt.config(state=tk.NORMAL)


	def on_resize(self, event):

		#Update to get the current values
		self.update()
		curr_size = (self.winfo_width(), self.winfo_height())

		if self.previous_size != curr_size:

			height = self.winfo_height() / 2 - 50
			width = self.winfo_width() / 2 -50

			for plot in self.plots.values():

				plot.canvas.get_tk_widget().config(width=width, height=height)
				plot.fast_update()

		self.previous_size = curr_size


	def start_listening(self):
		pass

	def update_plots(self):
		pass

	def add_GUI(self):
		pass

	def run(self):

		while rclpy.ok():

			if self.need_to_update:

				self.update_plots()


			self.update()
			self.update_idletasks()


	def on_destroy(self):

		self.destroy()
		self.destroy_node()
		rclpy.shutdown()
		self.spin_thread.join()