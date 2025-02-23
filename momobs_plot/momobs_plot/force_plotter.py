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



legs = ["LF", "RF", "LH", "RH"]
legs_names = ["LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"]
force_labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
comp_labels = ["GT X", "EST X", "GT Y", "EST Y", "GT Z", "EST Z"]
norm_label = ["Norm Error", "", "", "", "", ""]
comp_colors = ['#1f77b4', '#1f77b4', '#ff7f0e', '#ff7f0e', '#2ca02c', '#2ca02c']
comp_style = ['-', '--', '-', '--', '-', '--']


class ForcePlotter(PlotterBase):


	def __init__(self):

		PlotterBase.__init__(self, "ForcePlotter")

		mujoco_error = False
		anymal_error = False

		self.prev_mode = 0
		self.changed_axis = False
		self.prev_axis = 0

		self.forces = {	'LF_FOOT': [np.empty((6,0)), np.empty(0)], 
				 		'RF_FOOT': [np.empty((6,0)), np.empty(0)], 
						'LH_FOOT': [np.empty((6,0)), np.empty(0)], 
						'RH_FOOT': [np.empty((6,0)), np.empty(0)]}
		self.gt = {	'LF_FOOT': [np.empty((6,0)), np.empty(0)], 
					'RF_FOOT': [np.empty((6,0)), np.empty(0)], 
					'LH_FOOT': [np.empty((6,0)), np.empty(0)], 
					'RH_FOOT': [np.empty((6,0)), np.empty(0)]}
		self.gt_est_z = {	'LF_FOOT': [np.empty((6,0)), np.empty(0)], 
							'RF_FOOT': [np.empty((6,0)), np.empty(0)], 
							'LH_FOOT': [np.empty((6,0)), np.empty(0)], 
							'RH_FOOT': [np.empty((6,0)), np.empty(0)]}
		self.err = {'LF_FOOT': [np.empty((6,0)), np.empty(0)], 
			  		'RF_FOOT': [np.empty((6,0)), np.empty(0)], 
					'LH_FOOT': [np.empty((6,0)), np.empty(0)], 
					'RH_FOOT': [np.empty((6,0)), np.empty(0)]}
		self.mse = {'LF_FOOT': [np.empty((6,0)), np.empty(0)], 
			  		'RF_FOOT': [np.empty((6,0)), np.empty(0)], 
					'LH_FOOT': [np.empty((6,0)), np.empty(0)], 
					'RH_FOOT': [np.empty((6,0)), np.empty(0)]}
		
		# self.time = {'LF_FOOT': np.empty(0), 'RF_FOOT': np.empty(0), 'LH_FOOT': np.empty(0), 'RH_FOOT': np.empty(0)}
		self.norm = {	'LF_FOOT': np.empty(0), 
						'RF_FOOT': np.empty(0), 
						'LH_FOOT': np.empty(0), 
						'RH_FOOT': np.empty(0)}
		
		self.contacts = {'LF_FOOT': True, 'RF_FOOT': True, 'LH_FOOT': True, 'RH_FOOT': True}
		self.last_contacts = {'LF_FOOT': True, 'RF_FOOT': True, 'LH_FOOT': True, 'RH_FOOT': True}

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

		if mujoco_error and anymal_error:
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
			legs[0]:PlotContainer(self, f"Estimated Forces - {legs[0]}"),
			legs[1]:PlotContainer(self, f"Estimated Forces - {legs[1]}"),
			legs[2]:PlotContainer(self, f"Estimated Forces - {legs[2]}"),
			legs[3]:PlotContainer(self, f"Estimated Forces - {legs[3]}"),
		}

		self.plots[legs[0]].grid(row=1, column=0, padx=0, pady=0, 	columnspan=9,	sticky="w")
		self.plots[legs[1]].grid(row=2, column=0, padx=0, pady=0, 	columnspan=9,	sticky="w")
		self.plots[legs[2]].grid(row=1, column=9, padx=0, pady=0, 	columnspan=9,	sticky="w")
		self.plots[legs[3]].grid(row=2, column=9, padx=0, pady=0, 	columnspan=9,	sticky="w")


	def start_listening(self):
		
		self.listening = not self.listening
		self.show_torques_butt.config(state=tk.ACTIVE)	

		if self.listening:

			for plot in self.plots.values():

				plot.clear()

			self.forces = {	'LF_FOOT': [np.empty((6,0)), np.empty(0)], 
				 		'RF_FOOT': [np.empty((6,0)), np.empty(0)], 
						'LH_FOOT': [np.empty((6,0)), np.empty(0)], 
						'RH_FOOT': [np.empty((6,0)), np.empty(0)]}
			self.gt = {	'LF_FOOT': [np.empty((6,0)), np.empty(0)], 
						'RF_FOOT': [np.empty((6,0)), np.empty(0)], 
						'LH_FOOT': [np.empty((6,0)), np.empty(0)], 
						'RH_FOOT': [np.empty((6,0)), np.empty(0)]}
			self.err = {'LF_FOOT': [np.empty((6,0)), np.empty(0)], 
						'RF_FOOT': [np.empty((6,0)), np.empty(0)], 
						'LH_FOOT': [np.empty((6,0)), np.empty(0)], 
						'RH_FOOT': [np.empty((6,0)), np.empty(0)]}
			self.mse = {'LF_FOOT': [np.empty((6,0)), np.empty(0)], 
						'RF_FOOT': [np.empty((6,0)), np.empty(0)], 
						'LH_FOOT': [np.empty((6,0)), np.empty(0)], 
						'RH_FOOT': [np.empty((6,0)), np.empty(0)]}
			self.norm = {	'LF_FOOT': np.empty(0), 
						'RF_FOOT': np.empty(0), 
						'LH_FOOT': np.empty(0), 
						'RH_FOOT': np.empty(0)}
			self.gt_est_z = {	'LF_FOOT': [np.empty((6,0)), np.empty(0)], 
								'RF_FOOT': [np.empty((6,0)), np.empty(0)], 
								'LH_FOOT': [np.empty((6,0)), np.empty(0)], 
								'RH_FOOT': [np.empty((6,0)), np.empty(0)]}
			# self.time = {'LF_FOOT': np.empty(0), 'RF_FOOT': np.empty(0), 'LH_FOOT': np.empty(0), 'RH_FOOT': np.empty(0)}

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
			t = c.header.stamp.sec + c.header.stamp.nanosec * 1e-9
			self.gt[c.name][0] = np.hstack((self.gt[c.name][0], f))
			self.gt[c.name][1] = np.append(self.gt[c.name][1], t)
			self.gt_est_z[c.name][0] = np.hstack((self.gt_est_z[c.name][0], np.zeros((6,1))))
			self.gt_est_z[c.name][0][0][-1] = f[0][0]
			self.gt_est_z[c.name][0][2][-1] = f[1][0]
			self.gt_est_z[c.name][0][4][-1] = f[2][0]
			if self.gt[c.name][0].shape[1] > self.limit:
				self.gt[c.name][0] = self.gt[c.name][0][:,1:]
				self.gt[c.name][1] = self.gt[c.name][1][1:]

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
			t = est.header.stamp.sec + est.header.stamp.nanosec * 1e-9
			self.forces[est.names[j].data][0] = np.hstack((self.forces[est.names[j].data][0], force))
			self.forces[est.names[j].data][1] = np.append(self.forces[est.names[j].data][1], t)
			self.gt_est_z[est.names[j].data][0][1][-1] = force[0][0]
			self.gt_est_z[est.names[j].data][0][3][-1] = force[1][0]
			self.gt_est_z[est.names[j].data][0][5][-1] = force[2][0]
			if self.forces[est.names[j].data][0].shape[1] > self.limit:
				self.forces[est.names[j].data][0] = self.forces[est.names[j].data][0][:, 1:]
				self.forces[est.names[j].data][1] = self.forces[est.names[j].data][1][1:]
				self.gt_est_z[est.names[j].data][0] = self.gt_est_z[est.names[j].data][0][:, 1:]

		self.update_stats(ngt, nest)
		self.need_to_update = True
		

	def mujoco_callback(self, *msgs):
		
		if not self.listening:
			return
		
		[mujoco, est] = msgs

		ngt = {}
		nest = {}

		gt_contacts = {}

		for c in mujoco.contacts:			

			if c.object2_name in legs_names:

				f = np.array([
					c.contact_force.force.x,	
					c.contact_force.force.y,	
					c.contact_force.force.z,	
					c.contact_force.torque.x,	
					c.contact_force.torque.y,	
					c.contact_force.torque.z	
				]).reshape((6,1)) 

				gt_contacts[c.object2_name] = f
				# headers[c.object2_name] = c.header.stamp

		for l in legs_names:
			if not l in gt_contacts.keys():
				gt_contacts[l] = np.zeros((6,1))

		for l in legs:				
						
			f = gt_contacts[f'{l}_FOOT']

			ngt[f'{l}_FOOT'] = np.linalg.norm(f[0:2])
			t = 0

			self.gt[f'{l}_FOOT'][0] = np.hstack((self.gt[f'{l}_FOOT'][0], f))
			self.gt[f'{l}_FOOT'][1] = np.append(self.gt[f'{l}_FOOT'][1], t)
			self.gt_est_z[f'{l}_FOOT'][0] = np.hstack((self.gt_est_z[f'{l}_FOOT'][0], np.zeros((6,1))))
			self.gt_est_z[f'{l}_FOOT'][0][0][-1] = f[0][0]
			self.gt_est_z[f'{l}_FOOT'][0][2][-1] = f[1][0]
			self.gt_est_z[f'{l}_FOOT'][0][4][-1] = f[2][0]

			if self.gt[f'{l}_FOOT'][0].shape[1] > self.limit:

				self.gt[f'{l}_FOOT'][0] = self.gt[f'{l}_FOOT'][0][:,1:]
				self.gt[f'{l}_FOOT'][1] = self.gt[f'{l}_FOOT'][1][1:]

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
			t = est.header.stamp.sec + est.header.stamp.nanosec * 1e-9
			self.forces[est.names[j].data][0] = np.hstack((self.forces[est.names[j].data][0], force))
			self.forces[est.names[j].data][1] = np.append(self.forces[est.names[j].data][1], t)
			self.gt_est_z[est.names[j].data][0][1][-1] = force[0][0]
			self.gt_est_z[est.names[j].data][0][3][-1] = force[1][0]
			self.gt_est_z[est.names[j].data][0][5][-1] = force[2][0]
			if self.forces[est.names[j].data][0].shape[1] > self.limit:
				self.forces[est.names[j].data][0] = self.forces[est.names[j].data][0][:, 1:]
				self.forces[est.names[j].data][1] = self.forces[est.names[j].data][1][1:]
				self.gt_est_z[est.names[j].data][0] = self.gt_est_z[est.names[j].data][0][:, 1:]

		self.update_stats(ngt, nest)
		self.need_to_update = True
		



	def update_stats(self, ngt, nest):

		for i in range(4):

			self.err[f'{legs[i]}_FOOT'][0] = self.gt[f'{legs[i]}_FOOT'][0] - self.forces[f'{legs[i]}_FOOT'][0]
			self.err[f'{legs[i]}_FOOT'][1] = self.gt[f'{legs[i]}_FOOT'][1]

			mse = np.mean((self.gt[f'{legs[i]}_FOOT'][0] - self.forces[f'{legs[i]}_FOOT'][0])**2, axis=1).reshape((6,1))

			self.mse[f'{legs[i]}_FOOT'][0] = np.hstack((self.mse[f'{legs[i]}_FOOT'][0], np.sqrt(mse)))
			self.mse[f'{legs[i]}_FOOT'][1] = self.gt[f'{legs[i]}_FOOT'][1]

			self.norm[f'{legs[i]}_FOOT'] = np.append(self.norm[f'{legs[i]}_FOOT'], ngt[f'{legs[i]}_FOOT'] - nest[f'{legs[i]}_FOOT'])

			if self.mse[f'{legs[i]}_FOOT'][0].shape[1] > self.limit:

				self.mse[f'{legs[i]}_FOOT'][0] = self.mse[f'{legs[i]}_FOOT'][0][:, 1:]
				self.norm[f'{legs[i]}_FOOT'] = self.norm[f'{legs[i]}_FOOT'][1:]			
			

	
	def update_plots(self):

		labels = force_labels
		title = None

		if self.only_z.get() != self.prev_axis:
			self.prev_axis = self.only_z.get()
			self.changed_axis = True

		if self.prev_mode != self.mode.get() or self.changed_axis:
			self.prev_mode = self.mode.get()
			self.changed_axis = False
			for plot in self.plots.values():
				plot.clear()


		for i, plot in enumerate(self.plots.values()):				

			if self.mode.get() == 0:

				to_plot = self.forces[f'{legs[i]}_FOOT'][0]
				# time = self.forces[f'{legs[i]}_FOOT'][1]

			elif self.mode.get() == 1:

				to_plot = self.gt[f'{legs[i]}_FOOT'][0]
				# time = self.gt[f'{legs[i]}_FOOT'][1]
				title = f'Groundtruth forces - {legs[i]}'

			elif self.mode.get() == 2:

				to_plot = self.err[f'{legs[i]}_FOOT'][0]
				# time = self.err[f'{legs[i]}_FOOT'][1]
				title = f'Estimation error - {legs[i]}'

			elif self.mode.get() == 3:

				to_plot = self.mse[f'{legs[i]}_FOOT'][0]				
				# time = self.mse[f'{legs[i]}_FOOT'][1]
				title = f'RMSE - {legs[i]}'

			elif self.mode.get() == 4:

				to_plot = self.norm[f'{legs[i]}_FOOT']	
				l = to_plot.shape[0]
				to_plot = np.array([np.zeros(l), np.zeros(l), to_plot])		
				# time = self.mse[f'{legs[i]}_FOOT'][1]
				labels = norm_label
				title = f'Norm error - {legs[i]}'

			elif self.mode.get() == 5:

				to_plot = self.gt_est_z[f'{legs[i]}_FOOT'][0]
				title = f'Forces comparison - {legs[i]}'

				
			if self.show_torques.get():

				plot.update_plot(to_plot, labels, title=title)

			else:						

				if self.only_z.get():

					if self.mode.get() == 5:
						plot.update_plot(to_plot[-2:, :], comp_labels[-2:], title=title, color=comp_colors, style=comp_style)
					else:
						l = to_plot[2, :].shape
						only_z = np.array([np.zeros(l), np.zeros(l), to_plot[2, :]])
						plot.update_plot(only_z, labels[:3], title=title)

				else:
					
					if self.mode.get() == 5:
						plot.update_plot(to_plot, comp_labels, title=title, color=comp_colors, style=comp_style)		
			
					else:
			
						plot.update_plot(to_plot[:3, :], labels[:3],title=title)

		self.need_to_update = False



















def main(args=None):
	rclpy.init(args=args)
	plotter = ForcePlotter()
	plotter.run()


if __name__ == '__main__':
	main()