# main.py
from pylab import mpl

import matplotlib.pyplot as plt
import point
from mpl_toolkits.mplot3d import Axes3D
import point_map
import a_star

# 设置数据读取的路径以及工作表名称
data_path = "附件1：数据集1-终稿.xlsx"
data_sheet_name = 'data1'
# data_path = "附件2：数据集2-终稿.xlsx"
# data_sheet_name = 'data2'

# 创建所有点的Map
point_map = point_map.Map(data_path, data_sheet_name)

# 创建一个三维的绘图工程
plt.figure(figsize=(10, 10))
ax = plt.gca(projection='3d')
plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号

equal = point_map.point_list_numpy[:, 4] == 0
horizontal_point = point_map.point_list_numpy[equal]
x, y, z = horizontal_point[:, 1], horizontal_point[:, 2], horizontal_point[:, 3]
ax.scatter(x, y, z, c='y', label="水平校正点")  # 绘制水平校正点

equal = point_map.point_list_numpy[:, 4] == 1
horizontal_point = point_map.point_list_numpy[equal]
x, y, z = horizontal_point[:, 1], horizontal_point[:, 2], horizontal_point[:, 3]
ax.scatter(x, y, z, c='b', label="垂直校正点")  # 绘制水平校正点

end_index = len(point_map.point_list_numpy) - 1
ax.scatter(point_map.point_list_numpy[0][1], point_map.point_list_numpy[0][2], point_map.point_list_numpy[0][3], c='r',
           label="A点")  # 绘制A点
ax.scatter(point_map.point_list_numpy[end_index][1], point_map.point_list_numpy[end_index][2],
           point_map.point_list_numpy[end_index][3], c='r', label="B点")  # 绘制B点
ax.text(point_map.point_list_numpy[0][1] + 100, point_map.point_list_numpy[0][2] + 100,
        point_map.point_list_numpy[0][3] + 100, "A点")
ax.text(point_map.point_list_numpy[end_index][1] + 100, point_map.point_list_numpy[end_index][2] + 100,
        point_map.point_list_numpy[end_index][3] + 100, "B点")

# 设置坐标轴
ax.set_zlabel('Z')
ax.set_ylabel('Y')
ax.set_xlabel('X')
plt.legend(loc="best")
# plt.show()

# 创建起终点point对象
start_point = point.Point(point_map.point_list_numpy[0][0],
                          point_map.point_list_numpy[0][1],
                          point_map.point_list_numpy[0][2],
                          point_map.point_list_numpy[0][3],
                          point_map.point_list_numpy[0][4],
                          point_map.point_list_numpy[0][5])

end_point = point.Point(point_map.point_list_numpy[end_index][0],
                        point_map.point_list_numpy[end_index][1],
                        point_map.point_list_numpy[end_index][2],
                        point_map.point_list_numpy[end_index][3],
                        point_map.point_list_numpy[end_index][4],
                        point_map.point_list_numpy[end_index][5])

a_star = a_star.AStar(point_map, start_point, end_point)
a_star.RunAndSaveImage(ax, plt)
