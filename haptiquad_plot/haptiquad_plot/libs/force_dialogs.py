import tkinter as tk
from tkinter import ttk
import os
from PIL import Image
from io import BytesIO
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt

folders = ["est", "gt", "error", "rmse", "norm", "comp"]
ax_positions = [(0,0), (1,0), (0,1), (1,1), (2,0), (2,1)]

class SaveDialog(tk.Toplevel):

    def __init__(self, root, frozen_data):
        
        super().__init__(root)
        self.title("Save plots")
        # self.geometry("170x330")  
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

        ttk.Checkbutton(self, variable=self.single_image, text="Single image"). grid(row=3, column=0, padx=(15, 25), pady=5, columnspan=2, sticky="w")

        ttk.Checkbutton(self, variable=self.transparent, text="Transparent"). grid(row=4, column=0, padx=(15, 25), pady=5, columnspan=2, sticky="w")

        ttk.Button(self, text="Save figures", command=self.save).grid(row=6, column=0, padx=(15, 25), pady=10, sticky="we", columnspan=2)
        
        self.progress = ttk.Progressbar(self, orient='horizontal', length=130, mode='determinate', value=0)
        self.progress.grid(column=0, row=5, pady=5, columnspan=2, sticky="we", padx=5)


      


    def save_stats(self, filepath):

        if self.data == None:
            return

        with open(os.path.join(filepath, 'plot_stats.txt'), "w") as f:
            f.write("Plot stats:\n")
            for key in self.data['norm'].keys():

                f.write(f'-{key}:\n')

                avg_norm = np.mean(self.data['norm'][key])
                avg_rmse_x = np.mean(self.data['rmse'][key][0,:])
                avg_rmse_y = np.mean(self.data['rmse'][key][1,:])
                avg_rmse_z = np.mean(self.data['rmse'][key][2,:])
                min_norm = np.min(self.data['norm'][key])
                max_norm = np.max(self.data['norm'][key])
                min_rmse_x = np.min(self.data['rmse'][key][0,:])
                min_rmse_y = np.min(self.data['rmse'][key][1,:])
                min_rmse_z = np.min(self.data['rmse'][key][2,:])
                max_rmse_x = np.max(self.data['rmse'][key][0,:])
                max_rmse_y = np.max(self.data['rmse'][key][1,:])
                max_rmse_z = np.max(self.data['rmse'][key][2,:])
                last_val_rmse = self.data['rmse'][key][:,-1]

                f.write('   Norm:\n')
                f.write(f'      - Avg value: {avg_norm} N / {avg_norm/9.8} Kg\n')
                f.write(f'      - Min value: {min_norm} N / {min_norm/9.8} Kg\n')
                f.write(f'      - Max value: {max_norm} N / {max_norm/9.8} Kg\n')
                f.write('   RMSE:\n')
                f.write('       -X:\n')
                f.write(f'          - Avg value: {avg_rmse_x} N / {avg_rmse_x/9.8} Kg\n')
                f.write(f'          - Min value: {min_rmse_x} N / {min_rmse_x/9.8} Kg\n')
                f.write(f'          - Max value: {max_rmse_x} N / {max_rmse_x/9.8} Kg\n')
                f.write('       -Y:\n')
                f.write(f'          - Avg value: {avg_rmse_y} N / {avg_rmse_y/9.8} Kg\n')
                f.write(f'          - Min value: {min_rmse_y} N / {min_rmse_y/9.8} Kg\n')
                f.write(f'          - Max value: {max_rmse_y} N / {max_rmse_y/9.8} Kg\n')
                f.write('       -Z:\n')
                f.write(f'          - Avg value: {avg_rmse_z} N / {avg_rmse_z/9.8} Kg\n')
                f.write(f'          - Min value: {min_rmse_z} N / {min_rmse_z/9.8} Kg\n')
                f.write(f'          - Max value: {max_rmse_z} N / {max_rmse_z/9.8} Kg\n')
                f.write(f'      - Last value: {last_val_rmse} N / {last_val_rmse/9.8} Kg\n')                    



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
        new_path = os.path.join(path, f"force_output_{time_str}")

        os.makedirs(new_path)   

        for m in range(6):

            os.makedirs(os.path.join(new_path, folders[m]))
            single_fig, single_ax = plt.subplots(3, 2, constrained_layout=True)

            for i, plot in enumerate(self.root.plots.values()):

                to_plot, title, labels, colors, time, style, xlabel, ylabel = self.root.process_mode(i, m)
                fig, ax = plt.subplots(constrained_layout=True)
                fig.set_size_inches(float(self.w.get()), float(self.h.get()))
                single_fig.set_size_inches(float(self.w.get()) * 2, float(self.h.get()) * 3)

                if not to_plot.shape[1] == time.shape[0]:
                    diff = to_plot.shape[1] - time.shape[0]
                    if diff > 0:
                        to_plot = to_plot[:,:-diff]                        
                    else: 
                        time = time[:-diff]

                ax.grid(True)
                ax.set_title(self.title if title==None else title, fontsize=16)  
                for j in range(to_plot.shape[0]):
                    if not type(style) == type(None):
                        s = style[j]
                        c = colors[j]
                    else:
                        s = style
                        c = colors

                    if self.single_image.get():
                        single_ax[ax_positions[i][0], ax_positions[i][1]].grid(True)
                        single_ax[ax_positions[i][0], ax_positions[i][1]].set_title(self.title if title==None else title, fontsize=16)  
                        single_ax[ax_positions[i][0], ax_positions[i][1]].plot(time, to_plot[j], label=labels[j], color=c, linestyle=s, linewidth=3)
                        single_ax[ax_positions[i][0], ax_positions[i][1]].legend(loc="upper right")
                        single_ax[ax_positions[i][0], ax_positions[i][1]].set_xlabel("" if xlabel == None else xlabel, fontsize =14)
                        single_ax[ax_positions[i][0], ax_positions[i][1]].set_ylabel("" if ylabel == None else ylabel, fontsize = 14)
                    else:
                        ax.plot(time, to_plot[j], label=labels[j], color=c, linestyle=s, linewidth=3)
                        ax.legend(loc="upper right")                
                        ax.set_xlabel("" if xlabel == None else xlabel, fontsize =14)
                        ax.set_ylabel("" if ylabel == None else ylabel, fontsize = 14)

                if plot.autoscale:
                    ax.relim()
                    ax.autoscale()
                    single_ax[ax_positions[i][0], ax_positions[i][1]].relim()
                    single_ax[ax_positions[i][0], ax_positions[i][1]].autoscale()
                else:
                    ax.set_ylim(plot.range[0], plot.range[1])
                    single_ax[ax_positions[i][0], ax_positions[i][1]].set_ylim(plot.range[0], plot.range[1])

                ax.set_xlim(time[-1]-plot.x_range+0.2, time[-1])
                single_ax[ax_positions[i][0], ax_positions[i][1]].set_xlim(time[-1]-plot.x_range+0.2, time[-1])
                filename = os.path.join(new_path, folders[m], f"{list(self.root.plots.keys())[i]}.png")
                self.root.get_logger().info(f'Saving image: {filename}')
                if not self.single_image.get():
                    fig.savefig(filename, transparent=self.transparent.get(), dpi=200)
                plt.close(fig)
                self.progress.step(100 / (6 * len(self.root.plots.values())))
                self.root.update()
                self.root.update_idletasks()
            
            if self.single_image.get():
                single_filename = os.path.join(new_path, folders[m], f"{folders[m]}.png")
                single_fig.savefig(single_filename, transparent=self.transparent.get(), dpi=200)
            plt.close(single_fig)

        self.save_stats(new_path)

        self.destroy()

