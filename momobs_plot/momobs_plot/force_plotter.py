#!/usr/bin/env python3
from momobs_plot.libs import PlotterBase
from momobs_plot.libs import PlotContainer
import rclpy
from tkinter import ttk
import tkinter as tk
import numpy as np

from momobs_msgs.msg import EstimatedForces

from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from copy import deepcopy



force_labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
comp_labels = ["GT X", "EST X", "GT Y", "EST Y", "GT Z", "EST Z"]
norm_label = ["", "", "Norm Error", "", "", ""]
comp_colors = ['#1f77b4', '#14425f', '#ff7f0e', '#b0421e', '#2ca02c', '#4e781e']
comp_style = ['-', '--', '-', '--', '-', '--']

xlabels = ['Time [s]', 'Time [s]', 'Time [s]', 'Time [s]', 'Time [s]', 'Time [s]']
ylabels = ['Force [N]', 'Force [N]', 'Force [N]', 'Force [N]', 'Force [N]', 'Force [N]']

class ForcePlotter(PlotterBase):


	def __init__(self):

		PlotterBase.__init__(self, "ForcePlotter")

		self.declare_parameter("foot_suffix", "")
		self.foot_suffix = self.get_parameter("foot_suffix").get_parameter_value().string_value

		if self.foot_suffix == "":
			raise RuntimeError("Foot suffix not set")

		mujoco_error = False
		anymal_error = False
		gazebo_error = False

		self.prev_mode = 0
		self.changed_axis = False
		self.prev_axis = 0
		self.foot_names = []
		self.forces, self.gt, self.gt_est, self.err, self.rmse, self.time, self.norm = {}, {}, {}, {}, {}, {}, {}
		self.frozen_data = None
		
		for pref in self.legs_prefix:

			key = pref + '_' + self.foot_suffix
			self.forces[key] = np.empty((6,0))
			self.gt[key] = np.empty((6,0))
			self.gt_est[key] = np.empty((6,0))
			self.err[key] = np.empty((6,0))
			self.rmse[key] = np.empty((6,0))
			self.time[key] = np.empty(0)
			self.norm[key] = np.empty(0)
			self.foot_names.append(key)


		try:
			from anymal_msgs.msg import AnymalState
		except ImportError:
			anymal_error = True
			self.get_logger().warn("anymal_msgs was not found. The callback will not be enabled")

		try:
			from mujoco_msgs.msg import MujocoContacts
		except ImportError:
			mujoco_error = True
			self.get_logger().warn("mujoco_msgs was not found. The callback will not be enabled")

		try:
			from gazebo_msgs.msg import ContactsState
		except ImportError:
			gazebo_error = True
			self.get_logger().warn("gazebo_msgs was not found. The callback will not be enabled")

		if mujoco_error and anymal_error and gazebo_error:
			self.get_logger().error("No relevant message packages were found, no callback can be executed. Exiting..")
			raise RuntimeError
		
		self.est = Subscriber(self, EstimatedForces, '/momobs_ros2/estimated_forces')


		if not anymal_error:

			self.bag = Subscriber(self, AnymalState, "/state_estimator/anymal_state")
			self.bag_subs = [self.bag, self.est]
			self.bag_sync = TimeSynchronizer(self.bag_subs, queue_size=10)
			self.bag_sync.registerCallback(self.bag_callback)

		if not mujoco_error:

			mujoco_qos = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			history=QoSHistoryPolicy.KEEP_LAST,
			depth=1
			)

			self.mujoco_contacts = Subscriber(self, MujocoContacts, "/simulation/contacts", qos_profile=mujoco_qos)
			self.mujoco_subs = [self.mujoco_contacts, self.est]

			self.mujoco_sync = ApproximateTimeSynchronizer(self.mujoco_subs, queue_size=10, slop=0.05)
			self.mujoco_sync.registerCallback(self.mujoco_callback)

		if not gazebo_error:

			self.LF_contact = Subscriber(self, ContactsState, "/contact_force_sensors/LF")
			self.RF_contact = Subscriber(self, ContactsState, "/contact_force_sensors/RF")
			self.LH_contact = Subscriber(self, ContactsState, "/contact_force_sensors/LH")
			self.RH_contact = Subscriber(self, ContactsState, "/contact_force_sensors/RH")

			self.gazebo_subs = [self.LF_contact,
					   			self.RF_contact,
								self.LH_contact,
								self.RH_contact,
								self.est]

			self.gazebo_sync = ApproximateTimeSynchronizer(self.gazebo_subs, queue_size=50, slop=0.1)
			self.gazebo_sync.registerCallback(self.gazebo_callback)		

		self.init_from_params()



	def add_GUI(self):
		

		self.show_torques = tk.BooleanVar()

		self.show_torques_butt = ttk.Checkbutton(self, style='Toggle.TButton', text="Show Torques", variable=self.show_torques, width=7)
		self.show_torques_butt.grid(row = 0, column=6, padx = (5,5), pady = 10, sticky="we")

		ttk.Separator(self, orient=tk.VERTICAL).grid(row = 0, column=7, sticky="ns", pady = 5, padx = 1)

		self.mode = tk.IntVar()
		self.mode.set(2)

		ttk.Label(self, text="Mode:").grid(row = 0, column=8, sticky="ns", pady = 5, padx = (0, 0))

		self.show_est_butt = ttk.Radiobutton(self, style='Toggle.TButton', text="Estimate", variable=self.mode, width=4, value=0)
		self.show_est_butt.grid(row = 0, column=9, padx = (5,5), pady = 10, sticky="we")

		self.show_gt_butt = ttk.Radiobutton(self, style='Toggle.TButton', text="GT", variable=self.mode, width=4, value=1)
		self.show_gt_butt.grid(row = 0, column=10, padx = (5,5), pady = 10, sticky="we")

		self.show_err_butt = ttk.Radiobutton(self, style='Toggle.TButton', text="Error", variable=self.mode, width=4, value=2)
		self.show_err_butt.grid(row = 0, column=11, padx = (5,5), pady = 10, sticky="we")

		self.show_mse_butt = ttk.Radiobutton(self, style='Toggle.TButton', text="RMSE", variable=self.mode, width=4, value=3)
		self.show_mse_butt.grid(row = 0, column=12, padx = (5,5), pady = 10, sticky="we")

		self.show_norm_butt = ttk.Radiobutton(self, style='Toggle.TButton', text="Norm Err", variable=self.mode, width=4, value=4)
		self.show_norm_butt.grid(row = 0, column=13, padx = (5,5), pady = 10, sticky="we")

		self.show_both_butt = ttk.Radiobutton(self, style='Toggle.TButton', text="GT+EST", variable=self.mode, width=4, value=5)
		self.show_both_butt.grid(row = 0, column=14, padx = (5,5), pady = 10, sticky="we")

		self.only_z = tk.BooleanVar()
		self.only_z.set(False)
		self.only_z_butt = ttk.Checkbutton(self, text="Only Z", variable=self.only_z)
		self.only_z_butt.grid(row = 0, column=15, padx = (5,5), pady = 10, sticky="we")

		self.plots = {
			self.legs_prefix[0]:PlotContainer(self, f"Estimated Forces - {self.legs_prefix[0]}"),
			self.legs_prefix[1]:PlotContainer(self, f"Estimated Forces - {self.legs_prefix[1]}"),
			self.legs_prefix[2]:PlotContainer(self, f"Estimated Forces - {self.legs_prefix[2]}"),
			self.legs_prefix[3]:PlotContainer(self, f"Estimated Forces - {self.legs_prefix[3]}"),
		}

		self.plots[self.legs_prefix[0]].grid(row=1, column=0, padx=0, pady=0, 	columnspan=9,	sticky="w")
		self.plots[self.legs_prefix[1]].grid(row=2, column=0, padx=0, pady=0, 	columnspan=9,	sticky="w")
		self.plots[self.legs_prefix[2]].grid(row=1, column=9, padx=0, pady=0, 	columnspan=9,	sticky="w")
		self.plots[self.legs_prefix[3]].grid(row=2, column=9, padx=0, pady=0, 	columnspan=9,	sticky="w")


	def start_listening(self):
		
		self.listening = not self.listening
		self.show_torques_butt.config(state=tk.ACTIVE)	

		if self.listening:

			for plot in self.plots.values():

				plot.clear()

			for pref in self.legs_prefix:

				key = pref + '_' + self.foot_suffix
				self.forces[key] = np.empty((6,0))
				self.gt[key] = np.empty((6,0))
				self.gt_est[key] = np.empty((6,0))
				self.err[key] = np.empty((6,0))
				self.rmse[key] = np.empty((6,0))
				self.time[key] = np.empty(0)
				self.norm[key] = np.empty(0)

			self.show_torques_butt.config(state=tk.DISABLED)	

	def bag_callback(self, *msgs):
		
		if not self.listening:
			return
		[gt, est] = msgs
		ngt = {}
		nest = {}

		for c in gt.contacts:

			f = np.array([
					c.wrench.force.x,	
					c.wrench.force.y,	
					c.wrench.force.z,	
					c.wrench.torque.x,	
					c.wrench.torque.y,	
					c.wrench.torque.z	
				]).reshape((6,1)) 

			ngt[c.name] = np.linalg.norm(f[0:2])
			self.gt[c.name] = np.hstack((self.gt[c.name], f))
			self.gt_est[c.name] = np.hstack((self.gt_est[c.name], np.zeros((6,1))))
			self.gt_est[c.name][0][-1] = f[0][0]
			self.gt_est[c.name][2][-1] = f[1][0]
			self.gt_est[c.name][4][-1] = f[2][0]

			if self.gt[c.name].shape[1] > self.limit:
				self.gt[c.name] = self.gt[c.name][:,1:]

			t = c.header.stamp.sec + 1e-9 * c.header.stamp.nanosec
			if self.start_time == None:
				self.start_time = t			
			
			normalized_time = t-self.start_time
			self.time[c.name] = np.append(self.time[c.name], normalized_time)

		for j in range(4):

			force = np.array([	
					est.forces[j].force.x,
					est.forces[j].force.y,
					est.forces[j].force.z,
					est.forces[j].torque.x,
					est.forces[j].torque.y,
					est.forces[j].torque.z,
				]).reshape((6,1))
			
			nest[est.names[j].data] = np.linalg.norm(force[0:2])
			self.forces[est.names[j].data] = np.hstack((self.forces[est.names[j].data], force))
			self.gt_est[est.names[j].data][1][-1] = force[0][0]
			self.gt_est[est.names[j].data][3][-1] = force[1][0]
			self.gt_est[est.names[j].data][5][-1] = force[2][0]
			if self.forces[est.names[j].data].shape[1] > self.limit:
				self.forces[est.names[j].data] = self.forces[est.names[j].data][:, 1:]
				self.gt_est[est.names[j].data] = self.gt_est[est.names[j].data][:, 1:]

		self.update_stats(ngt, nest)
		

	def mujoco_callback(self, *msgs):
		
		if not self.listening:
			return
		
		[mujoco, est] = msgs

		ngt = {}
		nest = {}

		gt_contacts = {}

		for c in mujoco.contacts:			

			if c.object2_name in self.foot_names:

				f = np.array([
					c.contact_force.force.x,	
					c.contact_force.force.y,	
					c.contact_force.force.z,	
					c.contact_force.torque.x,	
					c.contact_force.torque.y,	
					c.contact_force.torque.z	
				]).reshape((6,1)) 

				gt_contacts[c.object2_name] = f

		for key in self.foot_names:
			if not key in gt_contacts.keys():
				gt_contacts[key] = np.zeros((6,1))

		for key in self.foot_names:				
						
			f = gt_contacts[key]

			ngt[key] = np.linalg.norm(f[0:2])
			
			t = mujoco.header.stamp.sec + 1e-9 * mujoco.header.stamp.nanosec
			if self.start_time == None:
				self.start_time = t			
			
			normalized_time = t-self.start_time
			self.time[key] = np.append(self.time[key], normalized_time)

			self.gt[key] = np.hstack((self.gt[key], f))
			self.gt_est[key] = np.hstack((self.gt_est[key], np.zeros((6,1))))
			self.gt_est[key][0][-1] = f[0][0]
			self.gt_est[key][2][-1] = f[1][0]
			self.gt_est[key][4][-1] = f[2][0]

			if self.gt[key].shape[1] > self.limit:

				self.gt[key] = self.gt[key][:,1:]

		for j in range(4):

			force = np.array([	
					est.forces[j].force.x,
					est.forces[j].force.y,
					est.forces[j].force.z,
					est.forces[j].torque.x,
					est.forces[j].torque.y,
					est.forces[j].torque.z,
				]).reshape((6,1))
			
			nest[est.names[j].data] = np.linalg.norm(force[0:2])
			self.forces[est.names[j].data] = np.hstack((self.forces[est.names[j].data], force))
			self.gt_est[est.names[j].data][1][-1] = force[0][0]
			self.gt_est[est.names[j].data][3][-1] = force[1][0]
			self.gt_est[est.names[j].data][5][-1] = force[2][0]
			if self.forces[est.names[j].data].shape[1] > self.limit:
				self.forces[est.names[j].data] = self.forces[est.names[j].data][:, 1:]
				self.gt_est[est.names[j].data] = self.gt_est[est.names[j].data][:, 1:]

		self.update_stats(ngt, nest)
		


	def process_gazebo_contact(self, msg, key, ngt):

		if len(msg.states) > 0:

			f = np.array([
						msg.states[-1].total_wrench.force.x,	
						msg.states[-1].total_wrench.force.y,	
						msg.states[-1].total_wrench.force.z,	
						msg.states[-1].total_wrench.torque.x,	
						msg.states[-1].total_wrench.torque.y,	
						msg.states[-1].total_wrench.torque.z	
					]).reshape((6,1)) 
			
		else:

			f = np.zeros(6).reshape((6,1))
		
		self.gt[key] = np.hstack((self.gt[key], f))
		ngt[key] = np.linalg.norm(f[0:2])

		self.gt_est[key] = np.hstack((self.gt_est[key], np.zeros((6,1))))
		self.gt_est[key][0][-1] = f[0][0]
		self.gt_est[key][2][-1] = f[1][0]
		self.gt_est[key][4][-1] = f[2][0]

		t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
		if self.start_time == None:
			self.start_time = t			
		
		normalized_time = t-self.start_time
		self.time[key] = np.append(self.time[key], normalized_time)

		if self.gt[key].shape[1] > self.limit:

			self.gt[key] = self.gt[key][:,1:]
		
		return ngt



	def gazebo_callback(self, *msgs):
		
		if not self.listening:
			return
		
		[LF, RF, LH, RH, est] = msgs

		ngt = {}
		nest = {}

		ngt = self.process_gazebo_contact(LF, self.foot_names[0], ngt)
		ngt = self.process_gazebo_contact(RF, self.foot_names[1], ngt)
		ngt = self.process_gazebo_contact(LH, self.foot_names[2], ngt)
		ngt = self.process_gazebo_contact(RH, self.foot_names[3], ngt)


		for j in range(4):

			force = np.array([	
					est.forces[j].force.x,
					est.forces[j].force.y,
					est.forces[j].force.z,
					est.forces[j].torque.x,
					est.forces[j].torque.y,
					est.forces[j].torque.z,
				]).reshape((6,1))
			
			nest[est.names[j].data] = np.linalg.norm(force[0:2])
			self.forces[est.names[j].data] = np.hstack((self.forces[est.names[j].data], force))
			self.gt_est[est.names[j].data][1][-1] = force[0][0]
			self.gt_est[est.names[j].data][3][-1] = force[1][0]
			self.gt_est[est.names[j].data][5][-1] = force[2][0]
			if self.forces[est.names[j].data].shape[1] > self.limit:
				self.forces[est.names[j].data] = self.forces[est.names[j].data][:, 1:]
				self.gt_est[est.names[j].data] = self.gt_est[est.names[j].data][:, 1:]

		self.update_stats(ngt, nest)





	def update_stats(self, ngt, nest):

		for i in range(4):

			key = self.foot_names[i]

			self.err[key] = self.gt[key] - self.forces[key]

			mse = np.mean((self.gt[key] - self.forces[key])**2, axis=1).reshape((6,1))

			self.rmse[key] = np.hstack((self.rmse[key], np.sqrt(mse)))

			self.norm[key] = np.append(self.norm[key], ngt[key] - nest[key])

			if self.rmse[key].shape[1] > self.limit:

				self.rmse[key] = self.rmse[key][:, 1:]
				self.norm[key] = self.norm[key][1:]		
				self.time[key] = self.time[key][1:]



	def process_mode(self, i):

		key = self.foot_names[i]
		mode = self.mode.get()
		labels = force_labels
		colors = None		
		style = None
		pause = self.pause.get()
		time = self.time[key] if not pause else self.frozen_data['time'][key]

		if mode == 0:
			
			to_plot = self.forces[key] if not pause else self.frozen_data['forces'][key]
			title = f'Estimated forces - {self.legs_prefix[i]}'

		elif mode == 1:

			to_plot = self.gt[key] if not pause else self.frozen_data['gt'][key]
			title = f'Groundtruth forces - {self.legs_prefix[i]}'

		elif mode == 2:

			to_plot = self.err[key] if not pause else self.frozen_data['error'][key]
			title = f'Estimation error - {self.legs_prefix[i]}'

		elif mode == 3:

			to_plot = self.rmse[key] if not pause else self.frozen_data['rmse'][key]			
			title = f'RMSE - {self.legs_prefix[i]}'

		elif mode == 4:

			to_plot = self.norm[key] if not pause else self.frozen_data['norm'][key]
			l = to_plot.shape[0]
			to_plot = np.array([np.zeros(l), np.zeros(l), to_plot])		
			labels = norm_label
			title = f'Norm error - {self.legs_prefix[i]}'

		elif mode == 5:

			to_plot = self.gt_est[key] if not pause else self.frozen_data['comp'][key]
			title = f'Forces comparison - {self.legs_prefix[i]}'
			colors = comp_colors
			labels = comp_labels
			style = comp_style

			if self.only_z.get() and not self.show_torques.get():

				to_plot = to_plot[-2:, :]
				labels = comp_labels[-2:]

		if not self.show_torques.get() and not self.only_z.get() and not mode == 5:

			to_plot = to_plot[:3,:]
			labels = labels[:3]

		if self.only_z.get() and not mode in [4,5]:

			l = to_plot[2, :].shape
			to_plot = np.array([np.zeros(l), np.zeros(l), to_plot[2, :]])
			labels = labels[:3]


		return to_plot, title, labels, colors, time, style
			

	
	def update_plots(self):


		if self.only_z.get() != self.prev_axis:
			self.prev_axis = self.only_z.get()
			self.changed_axis = True

		if self.prev_mode != self.mode.get() or self.changed_axis:
			self.prev_mode = self.mode.get()
			self.changed_axis = False
			for plot in self.plots.values():
				plot.clear()

		if self.pause.get() and type(self.frozen_data) == type(None):
			
			self.frozen_data = {}
			self.frozen_data['forces'] = deepcopy(self.forces)
			self.frozen_data['gt'] = deepcopy(self.gt)
			self.frozen_data['error'] = deepcopy(self.err)
			self.frozen_data['rmse'] = deepcopy(self.rmse)
			self.frozen_data['norm'] = deepcopy(self.norm)
			self.frozen_data['comp'] = deepcopy(self.gt_est)
			self.frozen_data['time'] = deepcopy(self.time)

		elif not self.pause.get() and type(self.frozen_data) == dict:

			del self.frozen_data
			self.frozen_data = None


		for i, plot in enumerate(self.plots.values()):		


			to_plot, title, labels, colors, time, style = self.process_mode(i)			

			if not time.shape[0] == to_plot.shape[1]:

				continue

			plot.update_plot(to_plot, labels, title=title, xlabel= xlabels[self.mode.get()], ylabel=ylabels[self.mode.get()], time = time, color=colors, style=style)






def main(args=None):
	rclpy.init(args=args)
	plotter = ForcePlotter()
	plotter.run()


if __name__ == '__main__':
	main()