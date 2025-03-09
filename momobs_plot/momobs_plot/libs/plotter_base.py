from ament_index_python.packages import get_package_share_directory
from momobs_plot.libs.dialogs import SaveDialog
import tkinter as tk
from tkinter import ttk
from rclpy.node import Node
import rclpy
from threading import Thread

from PIL import Image, ImageTk




class PlotterBase(Node, tk.Tk):


	def __init__(self, node_name):

		self.listening = False	
		self.autoscale = False
		self.autoscroll = True

		self.start_time = None

		Node.__init__(self, node_name)
		tk.Tk.__init__(self)

		self.protocol('WM_DELETE_WINDOW', self.on_destroy)
		self.geometry("1920x1080")
		tk.Tk.wm_title(self, node_name)

		self.declare_parameter('legs_prefix', [""])
		self.legs_prefix = self.get_parameter('legs_prefix').get_parameter_value().string_array_value

		if (len(self.legs_prefix) > 4):
			raise RuntimeError("Legs prefix are more than 4 - not supported")
		if self.legs_prefix == [""]:
			raise RuntimeError("Leg prefix not specified")

		theme_path = f"{get_package_share_directory('momobs_plot')}/theme/azure.tcl"
		save_icon_path = f"{get_package_share_directory('momobs_plot')}/res/save_icon.png"
		pause_icon_path = f"{get_package_share_directory('momobs_plot')}/res/pause_icon.png"

		self.save_icon = ImageTk.PhotoImage(Image.open(save_icon_path).resize((24,24)))
		self.pause_icon = ImageTk.PhotoImage(Image.open(pause_icon_path))

		self.tk.call("source", theme_path)
		self.tk.call("set_theme", "light")
		self.previous_size = (0, 0)

		menu_bar = tk.Menu(self)
		# File Menu
		file_menu = tk.Menu(menu_bar, tearoff=0)
		file_menu.add_command(label="Save figures", command=self.save_plots, image=self.save_icon, compound='left', accelerator="Ctrl+S")
		self.bind("<Control-s>", lambda event: self.save_plots()) 

		menu_bar.add_cascade(label="File", menu=file_menu)
		self.config(menu=menu_bar)

		self.scroll_range = tk.StringVar()


		self.declare_parameter('x_lim', 10.0)
		self.scroll_range.set(str(self.get_parameter('x_lim').get_parameter_value().double_value))


		self.declare_parameter('memory_limit', 1000)
		self.limit = self.get_parameter('memory_limit').get_parameter_value().integer_value

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

		self.pause = tk.BooleanVar()
		self.pause.set(False)
		self.pause_button = ttk.Checkbutton(self, style='Toggle.TButton', image=self.pause_icon, variable=self.pause)
		self.pause_button.grid(column=16, row=0, pady=10)

		self.plots = []

		self.spin_thread = Thread(target=rclpy.spin, args=(self,), daemon=True)
		self.spin_thread.start()

		self.add_GUI()

		self.set_scroll_range()

		self.bind("<Configure>", self.on_resize) 
		self.on_resize(None)

		self.rate = self.create_rate(60)

	
	def init_from_params(self):

		self.declare_parameter('autoscale', True)
		self.autoscale_from_start = self.get_parameter('autoscale').get_parameter_value().bool_value

		self.declare_parameter('listening', False)
		self.listening_from_start = self.get_parameter('listening').get_parameter_value().bool_value

		if self.autoscale_from_start:
			self.autoscale_butt.invoke()

		if self.listening_from_start:
			self.listen.invoke()


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

			self.update_plots()
			self.update()
			self.update_idletasks()
			self.rate.sleep()


	def on_destroy(self):

		self.destroy()
		self.destroy_node()
		rclpy.shutdown()
		self.spin_thread.join()



	def save_plots(self):

		SaveDialog(self)



		