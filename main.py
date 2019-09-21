# main.py

import numpy as np
import matplotlib.pyplot as plt
import xlrd
import point
from mpl_toolkits.mplot3d import Axes3D

import random_map
import a_star

excel = xlrd.open_workbook("G:/研究生课程相关/研究生数学建模/2019年中国研究生数学建模竞赛F题/a-star/附件1：数据集1-终稿.xlsx")
sheet = excel.sheet_by_name('data1')
rows = sheet.nrows  # 获取行数
cols = sheet.ncols  # 获取列数
data = np.zeros(shape=(rows - 2, cols))
row_index = 0
for r in range(2, rows):  # 读取每一行的数据
    r_values = sheet.row_values(r)
    p = point.Point(r_values[0], r_values[1], r_values[2], r_values[3], r_values[4], r_values[5])
    data[r - 2] = [r_values[0], r_values[1], r_values[2], r_values[3], r_values[4], r_values[5]]

plt.figure(figsize=(5, 5))

# map = random_map.RandomMap()

# ax = plt.gca()
# data = np.random.randint(0, 255, size=[40, 40, 40])

x, y, z = data[:, 1], data[:, 2], data[:, 3]
ax = plt.gca(projection='3d')  # 创建一个三维的绘图工程
#  将数据点分成三部分画，在颜色上有区分度
ax.scatter(x, y, z, c='y')  # 绘制数据点
# ax.scatter(x[10:20], y[10:20], z[10:20], c='r')
# ax.scatter(x[30:40], y[30:40], z[30:40], c='g')

ax.set_zlabel('Z')  # 坐标轴
ax.set_ylabel('Y')
ax.set_xlabel('X')

# plt.show()

# #画地图
# for i in range(map.size):
#     for j in range(map.size):
#         if map.IsObstacle(i,j):
#             rec = Rectangle((i, j), width=1, height=1, color='gray')
#             ax.add_patch(rec)
#         else:
#             rec = Rectangle((i, j), width=1, height=1, edgecolor='gray', facecolor='w')
#             ax.add_patch(rec)
#
# #画起点终点
# rec = Rectangle((0, 0), width = 1, height = 1, facecolor='b')
# ax.add_patch(rec)
#
# rec = Rectangle((map.size-1, map.size-1), width = 1, height = 1, facecolor='r')
# ax.add_patch(rec)
map = random_map.Map()
start_point = point.Point(map.point_list_numpy[0][0],
                          map.point_list_numpy[0][1],
                          map.point_list_numpy[0][2],
                          map.point_list_numpy[0][3],
                          map.point_list_numpy[0][4],
                          map.point_list_numpy[0][5])
end_index = len(map.point_list_numpy) -1
end_point = point.Point(map.point_list_numpy[end_index][0],
                          map.point_list_numpy[end_index][1],
                          map.point_list_numpy[end_index][2],
                          map.point_list_numpy[end_index][3],
                          map.point_list_numpy[end_index][4],
                          map.point_list_numpy[end_index][5])
a_star = a_star.AStar(map, start_point, end_point)
a_star.RunAndSaveImage(ax, plt)
