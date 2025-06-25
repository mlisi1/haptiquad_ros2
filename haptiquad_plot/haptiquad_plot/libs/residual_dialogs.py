import tkinter as tk
from tkinter import ttk
import os
from PIL import Image
from io import BytesIO
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt

ax_positions = [(0,0), (1,0), (0,1), (1,1), (2,0), (2,1)]

class SaveDialog(tk.Toplevel):

    def __init__(self, root, frozen_data):
        
        super().__init__(root)
        self.title("Save plots")
        self.resizable(tk.FALSE, tk.FALSE)
        self.root = root
        self.data = frozen_data

        self.grab_set()
        self.transient(root)


        self.single_image = tk.BooleanVar()
        self.single_image.set(False)
        
        self.transparent = tk.BooleanVar()
        self.transparent.set(False)

        self.h = tk.StringVar()
        self.w = tk.StringVar()
        self.h.set("4.8")
        self.w.set("6.4")

        ttk.Label(self, text="Figure Size:").grid(row=0, column=0, sticky="w", pady=10, padx=(10, 5))

        ttk.Label(self, text="Height:").grid(row=1, column=0, pady=5, sticky="w", padx=(10,5))
        ttk.Spinbox(self, from_=1, to=2000, increment=1, textvariable=self.h, width=10).grid(row=1, column=1, padx=(5,10), sticky="w", pady=5)

        ttk.Label(self, text="Width:").grid(row=2, column=0, pady=5, sticky="w", padx=(10,5))
        ttk.Spinbox(self, from_=1, to=2000, increment=1, textvariable=self.w, width=10).grid(row=2, column=1, padx=(5,10), sticky="w", pady=5)

        # ttk.Checkbutton(self, variable=self.single_image, text="Single image"). grid(row=3, column=0, padx=(15, 25), pady=5, columnspan=2, sticky="w")

        ttk.Checkbutton(self, variable=self.transparent, text="Transparent"). grid(row=4, column=0, padx=(15, 25), pady=5, columnspan=2, sticky="w")

        ttk.Button(self, text="Save figures", command=self.save).grid(row=6, column=0, padx=(15, 25), pady=10, sticky="we", columnspan=2)
        
        self.progress = ttk.Progressbar(self, orient='horizontal', length=130, mode='determinate', value=0)
        self.progress.grid(column=0, row=5, pady=5, columnspan=2, sticky="we", padx=5)



    def disable_all_widgets(self):
        for child in self.winfo_children():
            try:
                child.configure(state='disabled')
            except tk.TclError:
                pass  


    def save(self):

        self.disable_all_widgets()
        path = tk.filedialog.askdirectory(title="Choose a Directory")

        now = datetime.now()
        time_str = now.strftime("%Y-%m-%d_%H-%M-%S")
        new_path = os.path.join(path, f"residual_output_{time_str}")

        os.makedirs(new_path)
        os.makedirs(os.path.join(new_path, "internal"))
        os.makedirs(os.path.join(new_path, "external"))

        for i, key in enumerate(self.root.plots.keys()):

            plot = self.root.plots[key]

            if not key == "External":

                to_plot, time, labels, title = self.root.proces_mode_int(i, False)
                fig, ax = plt.subplots(constrained_layout=True)
                fig.set_size_inches(float(self.w.get()), float(self.h.get()))

                if not to_plot.shape[1] == time.shape[0]:
                    diff = to_plot.shape[1] - time.shape[0]
                    if diff > 0:
                        to_plot = to_plot[:,:-diff]                        
                    else: 
                        time = time[:-diff]

                ax.grid(True)
                ax.set_title(title, fontsize=16) 

                for j in range(to_plot.shape[0]):
                    ax.plot(time, to_plot[j], label=labels[j], linewidth=3)

                ax.legend(loc="upper right")                
                ax.set_xlabel("Time [s]", fontsize =14)
                ax.set_ylabel("Momentum [Nm]", fontsize = 14)

                if plot.autoscale:
                    ax.relim()
                    ax.autoscale()
                else:
                    ax.set_ylim(plot.range[0], plot.range[1])

                ax.set_xlim(time[-1]-plot.x_range+0.2, time[-1])
                filename = os.path.join(new_path, "internal", f"{key}.png")
                self.root.get_logger().info(f'Saving image: {filename}')
                fig.savefig(filename, transparent=self.transparent.get(), dpi=200)

            else:

                to_plot, time, labels, title = self.root.proces_mode_ext(False)
                fig, ax = plt.subplots(constrained_layout=True)
                fig.set_size_inches(float(self.w.get()), float(self.h.get()))

                if not to_plot.shape[1] == time.shape[0]:
                    diff = to_plot.shape[1] - time.shape[0]
                    if diff > 0:
                        to_plot = to_plot[:,:-diff]                        
                    else: 
                        time = time[:-diff]

                ax.grid(True)
                ax.set_title(title, fontsize=16) 

                for j in range(to_plot.shape[0]):
                    ax.plot(time, to_plot[j], label=labels[j], linewidth=3)

                ax.legend(loc="upper right")                
                ax.set_xlabel("Time [s]", fontsize =14)
                ax.set_ylabel("Momentum [Nm]", fontsize = 14)

                if plot.autoscale:
                    ax.relim()
                    ax.autoscale()
                else:
                    ax.set_ylim(plot.range[0], plot.range[1])
                ax.set_xlim(time[-1]-plot.x_range+0.2, time[-1])
                filename = os.path.join(new_path, "external", f"{key}.png")
                self.root.get_logger().info(f'Saving image: {filename}')
                fig.savefig(filename, transparent=self.transparent.get(), dpi=200)

            self.progress.step(100 / 5)
            self.root.update()
            self.root.update_idletasks()

        self.destroy()

