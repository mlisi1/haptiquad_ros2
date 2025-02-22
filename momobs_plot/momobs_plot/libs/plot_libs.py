import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use("TkAgg")
import tkinter as tk
from tkinter import ttk



		



#=============== PLOT CONTAINER ====================
#Frame used to contain Matplotlibs tk wrapper; handles low level plot functions
class PlotContainer(ttk.Frame):

	def __init__(self, container, title, **args):

		#Initialize frame and figure
		tk.Frame.__init__(self, container, **args)
		self.fig, self.ax = plt.subplots()  
		self.ax.grid(True)   

		self.fig.subplots_adjust(0.08, 0.104, 0.968, 0.88, 0.2, 0.165)

		#Initialize Matplotlib wrappers and place them
		self.canvas = FigureCanvasTkAgg(self.fig, master = self)
		self.canvas.draw()
		self.canvas.get_tk_widget().pack(side=tk.BOTTOM)       
		self.canvas._tkcanvas.pack(side=tk.TOP, expand =True)
		self.title = title      
		self.ax.set_title(self.title)   

		self.first_time = True
		self.lines = []

		self.autoscale = False
		self.range = (-5,5)
		self.autoscroll_x = True
		self.x_range = 100


	#Clears the plot
	def clear(self):

		#Clear plot and draw it
		self.ax.clear()
		self.canvas.draw()
		self.lines = []
		self.first_time = True
		self.ax.grid(True)
		self.ax.set_ylim(self.range[0], self.range[1])



	#Updates the plot with the scalars data
	def update_plot(self, data, label_names, time = None):

		#Initialize plot limit array
		limit_values = []
		array_size = len(data[0])

		#Clear plot and reset plot relative arrays
		# self.ax.clear()     		

		if type(time) == type(None):
			x_values = list([i for i in range(array_size)])	
		else:
			x_values = time

		

		if self.first_time:

			for i in range(len(data)):

				label = label_names[i]

				line, = self.ax.plot(x_values, data[i,:], label=label)
				self.lines.append(line)

				
			self.ax.legend(loc="upper right")
			# self.canvas.draw()

			self.canvas.draw()  # Ensure the canvas is fully drawn
			self.ax.set_title(self.title)    

			self.first_time = False
			

		else:
		
			for i in range(len(data)):
				self.lines[i].set_ydata(data[i,:])
				self.lines[i].set_xdata(x_values)
				self.ax.draw_artist(self.lines[i])
			# self.ax.draw_artist(self.lines[i])
			# self.fig.canvas.blit(self.ax.bbox)


			if self.autoscale:

				self.ax.relim()
				self.ax.autoscale()
				# self.canvas.draw()

			else:

				self.ax.set_ylim(self.range[0], self.range[1])


			if self.autoscroll_x:

				self.ax.set_xlim(0, array_size)

			else:

				self.ax.set_xlim((array_size-self.x_range), array_size)

			self.canvas.draw()


	#Fast update method; it only redraws the plot;
	#>used when the scalars haven't changed but plot needs to be redrawn
	def fast_update(self):

		self.canvas.draw()
		



