# random_map.py

import numpy as np
from scipy.io import loadmat
import point
import xlrd

class Map:
    def __init__(self):
        excel = xlrd.open_workbook("G:/研究生课程相关/研究生数学建模/2019年中国研究生数学建模竞赛F题/a-star/附件2：数据集2-终稿.xlsx")
        sheet = excel.sheet_by_name('data2')
        rows = sheet.nrows  # 获取行数
        cols = sheet.ncols  # 获取列数
        data = np.zeros(shape=(rows - 2, cols))
        row_index = 0
        for r in range(2, rows):  # 读取每一行的数据
            r_values = sheet.row_values(r)
            p = point.Point(r_values[0], r_values[1], r_values[2], r_values[3], r_values[4], r_values[5])
            data[r - 2] = [r_values[0], r_values[1], r_values[2], r_values[3], r_values[4], r_values[5]]

        mat_path = "G:/研究生课程相关/研究生数学建模/2019年中国研究生数学建模竞赛F题/pre_data.mat"
        mat_data = loadmat(mat_path)

        self.point_list_numpy = data
        self.pre_data_numpy = mat_data['pre_data']

