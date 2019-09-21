# point_map.py
import numpy as np
import point
import xlrd
from scipy.io import loadmat


class Map:
    def __init__(self, data_path, data_sheet_name):
        excel = xlrd.open_workbook(data_path)
        sheet = excel.sheet_by_name(data_sheet_name)
        rows = sheet.nrows  # 获取行数
        cols = sheet.ncols  # 获取列数
        data = np.zeros(shape=(rows - 2, cols))
        row_index = 0
        for r in range(2, rows):  # 读取每一行的数据
            r_values = sheet.row_values(r)
            p = point.Point(r_values[0], r_values[1], r_values[2], r_values[3], r_values[4], r_values[5])
            data[r - 2] = [r_values[0], r_values[1], r_values[2], r_values[3], r_values[4], r_values[5]]

        self.point_list_numpy = data

        # mat_path = "pre_data.mat"
        # mat_data = loadmat(mat_path)
        # self.pre_data_numpy = mat_data['pre_data']
