import tkinter as tk
from tkinter import ttk
import os
from PIL import Image
from io import BytesIO



class SaveDialog(tk.Toplevel):

    def __init__(self, root):
        
        super().__init__(root)
        self.title("Save plots")
        # self.geometry("170x330")  
        self.resizable(tk.FALSE, tk.FALSE)
        self.root = root

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


                merged_image.save(path)



            else:

                for i, plot in enumerate(self.root.plots.values()):

                    plot.fig.savefig(os.path.join(path,list(self.root.plots.keys())[i]), transparent=self.transparent.get())

        else:

            for i, plot in enumerate(self.root.plots.values()):

                if self.to_save[i].get():

                    plot.fig.savefig(os.path.join(path, list(self.root.plots.keys())[i]), transparent=self.transparent.get())




