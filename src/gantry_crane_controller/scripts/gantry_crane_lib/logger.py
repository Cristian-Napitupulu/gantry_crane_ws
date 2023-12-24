import os
from datetime import datetime
import matplotlib.pyplot as plt
import pandas as pd
import time


class Logger:
    def __init__(self, parent_folder_path):
        self.parent_folder_path = parent_folder_path
        self.buffers = {}

    def add_to_buffer(self, variable_name, value):
        if variable_name not in self.buffers:
            self.buffers[variable_name] = []
        self.buffers[variable_name].append(value)

    def create_folder(self):
        folder_name = datetime.now().strftime("%Y-%m-%d/%H-%M-%S")
        self.folder_path = os.path.join(self.parent_folder_path, folder_name)
        os.makedirs(self.folder_path)

    def write_buffers_to_excel(self, file_name):
        self.create_folder()
        file_path = os.path.join(self.folder_path, file_name)
        data = pd.DataFrame(self.buffers)
        data.to_excel(file_path)

    def create_plot(self, variable_name, x_variable_name, y_variable_name):
        x = self.buffers[x_variable_name]
        y = self.buffers[y_variable_name]
        plt.plot(x, y)
        plt.xlabel(x_variable_name)
        plt.ylabel(y_variable_name)
        plt.title(variable_name)
        plot_path = os.path.join(self.folder_path, variable_name + ".png")
        plt.savefig(plot_path)
        plt.close()
    
    def begin(self):
        self.start_time = time.time()