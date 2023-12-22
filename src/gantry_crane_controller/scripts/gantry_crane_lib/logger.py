import os
from datetime import datetime
import matplotlib.pyplot as plt
import pandas as pd


class Logger():
    def __init__(self, parent_folder_path):
        self.parent_folder_path = parent_folder_path
        self.buffers = {}

    def create_buffer(self, variable_name):
        self.buffers[variable_name] = []

    def add_to_buffer(self, variable_name, value):
        if variable_name not in self.buffers:
            self.create_buffer(variable_name)
        self.buffers[variable_name].append(value)

    def write_buffers_to_excel(self, file_name):
        file_path = os.path.join(self.parent_folder_path, file_name)
        workbook = xlsxwriter.Workbook(file_path)
        worksheet = workbook.add_worksheet()
        row = 0
        for variable_name in self.buffers:
            worksheet.write(row, 0, variable_name)
            column = 1
            for value in self.buffers[variable_name]:
                worksheet.write(row, column, value)
                column += 1
            row += 1
        workbook.close()