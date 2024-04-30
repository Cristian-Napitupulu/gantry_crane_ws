import os
from datetime import datetime
import matplotlib.pyplot as plt
import pandas as pd
import time


class Logger:
    def __init__(self, parent_folder_path):
        self.parent_folder_path = parent_folder_path
        self.buffers = {}
        self.reset_timer()
        self.create_folder()

    def add_to_buffer(self, variable_name, value):
        if variable_name not in self.buffers:
            self.buffers[variable_name] = []
        self.buffers[variable_name].append(value)

    def create_folder(self):
        folder_name = datetime.now().strftime("%Y-%m-%d/%H-%M-%S")
        self.folder_path = os.path.join(self.parent_folder_path, folder_name)

    def write_buffers_to_excel(self, file_name, sheet_name='Sheet1'):
        if not os.path.exists(self.folder_path):
            os.makedirs(self.folder_path)

        file_path = os.path.join(self.folder_path, file_name)
        
        # Check if the file already exists
        if os.path.isfile(file_path):
            # If it exists, open the Excel file in append mode
            with pd.ExcelWriter(file_path, engine='openpyxl', mode='a') as writer:
                existing_sheets = writer.book.sheetnames
                
                # Generate a new sheet name if the specified one already exists
                new_sheet_name = sheet_name
                i = 1
                while new_sheet_name in existing_sheets:
                    new_sheet_name = f'{sheet_name}_{i}'
                    i += 1
                
                # Write the DataFrame to the new sheet
                pd.DataFrame(self.buffers).to_excel(writer, index=False, sheet_name=new_sheet_name)
        else:
            # If the file doesn't exist, create a new Excel file with the specified sheet name
            pd.DataFrame(self.buffers).to_excel(file_path, index=False, sheet_name=sheet_name)

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

    def reset_timer(self):
        self.start_time = time.time()

    def get_time_sec(self):
        return time.time() - self.start_time
    
    def reset_buffers(self):
        self.buffers = {}

    def create_live_plot_from_buffers(self, x_variable_name, y_variable_name):
        x = self.buffers[x_variable_name]
        y = self.buffers[y_variable_name]
        plt.plot(x, y)
        plt.xlabel(x_variable_name)
        plt.ylabel(y_variable_name)
        plt.title(f"{y_variable_name} vs {x_variable_name}")
        plt.pause(0.01)