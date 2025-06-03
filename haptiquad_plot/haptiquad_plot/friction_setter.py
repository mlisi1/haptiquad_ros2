#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tkinter as tk
from ament_index_python.packages import get_package_share_directory
from threading import Thread
from haptiquad_msgs.msg import FrictionParameters
from tkinter import ttk




class FrictionSetter(Node, tk.Tk):

	def __init__(self):

		Node.__init__(self, "FrictionSetter")
		tk.Tk.__init__(self)

		self.protocol('WM_DELETE_WINDOW', self.on_destroy)
		# self.geometry("200x300")
		tk.Tk.wm_title(self, "FrictionSetter")

		theme_path = f"{get_package_share_directory('haptiquad_plot')}/theme/azure.tcl"

		self.tk.call("source", theme_path)
		self.tk.call("set_theme", "light")

		self.friction_publisher = self.create_publisher(FrictionParameters, '/haptiquad_ros2/friction', 10)
		self.friction_msg = FrictionParameters()

		self.use_friction = tk.BooleanVar()
		self.use_friction.set(False)

		self.F_s = tk.StringVar()
		self.F_s.set("0.0")

		self.F_c = tk.StringVar()
		self.F_c.set("0.0")


		ttk.Checkbutton(self, text="Use Friction", variable=self.use_friction, style='Toggle.TButton').grid(row=0, column=0, padx=10, pady=10, columnspan=2)
		ttk.Label(self, text="Fs:").grid(row=1, column=0, padx=10, pady=10)
		ttk.Spinbox(self, from_=0.0, to=10000.0, increment=0.01, textvariable=self.F_s).grid(row=1, column=1, padx=10, pady=10)

		ttk.Label(self, text="Fc:").grid(row=2, column=0, padx=10, pady=10)
		ttk.Spinbox(self, from_=0.0, to=10000.0, increment=0.01, textvariable=self.F_c).grid(row=2, column=1, padx=10, pady=10)

		ttk.Separator(self, orient=tk.HORIZONTAL).grid(row=3, column=0, columnspan=2, sticky='we', padx=5, pady=10)

		ttk.Button(self, text="Send parameters", command=self.send_friction).grid(row=4, column=0, padx=10, pady=10, columnspan=2)

		self.spin_thread = Thread(target=rclpy.spin, args=(self,), daemon=True)
		self.spin_thread.start()



	def send_friction(self):

		self.friction_msg.use_friction.data = self.use_friction.get()
		self.friction_msg.f_s.data = float(self.F_s.get())
		self.friction_msg.f_c.data = float(self.F_c.get())

		self.friction_publisher.publish(self.friction_msg)


		

	def run(self):

		while rclpy.ok():

			self.update()
			self.update_idletasks()





	def on_destroy(self):

		self.destroy()
		self.destroy_node()
		rclpy.shutdown()
		self.spin_thread.join()








def main(args=None):
	rclpy.init(args=args)
	plotter = FrictionSetter()
	plotter.run()


if __name__ == '__main__':
	main()