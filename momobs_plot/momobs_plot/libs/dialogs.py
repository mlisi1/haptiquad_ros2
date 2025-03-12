import tkinter as tk
from tkinter import ttk
import os
from PIL import Image
from io import BytesIO
import numpy as np



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

        self.save_all = tk.BooleanVar()
        self.save_all.set(False)

        self.single_image = tk.BooleanVar()
        self.single_image.set(False)
        
        self.transparent = tk.BooleanVar()
        self.transparent.set(False)


        self.to_save = []
        self.checkbuttons = []

        self.all = ttk.Checkbutton(self, variable=self.save_all, text="Save all plots", command=self.toggle_save_all)
        self.all.grid(row=0, column=0, padx=(15, 25), pady=(10,5))

        ttk.Checkbutton(self, variable=self.single_image, text="Single image", command=self.toggle_single_image). grid(row=1, column=0, padx=(15, 25), pady=(5, 15))

        ttk.Checkbutton(self, variable=self.transparent, text="Transparent"). grid(row=2, column=0, padx=(15, 25), pady=(5, 15))

        ttk.Label(self, text="Plots to save:").grid(row=3, column=0, padx=(15, 25), pady=5, sticky='w')

        for i, key in enumerate(root.plots.keys()):

            self.to_save.append(tk.BooleanVar())
            self.checkbuttons.append(ttk.Checkbutton(self, text=key, variable=self.to_save[i]))
            self.checkbuttons[i].grid(column=0, row=4+i, padx=(15, 25), pady=5, sticky='w')

        ttk.Button(self, text="Save figures", command=self.save).grid(row=4+len(root.plots.keys()), column=0, padx=(15, 25), pady=10, sticky="we")


    def toggle_single_image(self):

        for i, c in enumerate(self.checkbuttons):

            self.to_save[i].set(self.single_image.get())
            c.config(state=tk.NORMAL if not self.single_image.get() else tk.DISABLED)

        self.save_all.set(self.single_image.get())
        self.all.config(state=tk.NORMAL if not self.single_image.get() else tk.DISABLED)       
      


    def toggle_save_all(self):

        for i, c in enumerate(self.checkbuttons):

            self.to_save[i].set(self.save_all.get())
            c.config(state=tk.NORMAL if not self.save_all.get() else tk.DISABLED)

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



    def save(self):

        if self.single_image.get():

            path = tk.filedialog.asksaveasfilename(
                defaultextension=".png",  # Default file extension
                filetypes=[("PNG Files", "*.png"), ("All Files", "*.*")],  # Allowed file types
                title="Save File As"
            )

        else:

            path = tk.filedialog.askdirectory(title="Choose a Directory")

        if self.save_all.get():

            self.save_stats(path)

            if self.single_image.get():
                
                images = []

                for i, plot in enumerate(self.root.plots.values()):

                    buffer = BytesIO()
                    plot.fig.savefig(buffer, transparent=self.transparent.get(), dpi = 300)
                    buffer.seek(0)
                    images.append(Image.open(buffer))

                width, height = images[0].size

                if len(images) == 5:

                    ext_width, ext_height = images[4].size
                    merged_image = Image.new("RGB", (width * 2, height * 2 + ext_height))          


                else:

                    merged_image = Image.new("RGB", (width * 2, height * 2))

                # Paste images into the merged image
                merged_image.paste(images[0], (0, 0))
                merged_image.paste(images[1], (width, 0))
                merged_image.paste(images[2], (0, height))
                merged_image.paste(images[3], (width, height))

                if len(images) == 5:

                    merged_image.paste(images[4], (-(ext_width-2*width), 2*height))

                pass
                merged_image.save(path)



            else:

                pass
                for i, plot in enumerate(self.root.plots.values()):

                    plot.fig.savefig(os.path.join(path,list(self.root.plots.keys())[i]), transparent=self.transparent.get())

        else:

            pass
            for i, plot in enumerate(self.root.plots.values()):

                if self.to_save[i].get():

                    plot.fig.savefig(os.path.join(path, list(self.root.plots.keys())[i]), transparent=self.transparent.get())

        
        self.destroy()



