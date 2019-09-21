# main.py
import matplotlib.pyplot as plt
import point
from mpl_toolkits.mplot3d import Axes3D
import point_map
import a_star

# 设置数据读取的路径以及工作表名称
data_path = "附件2：数据集2-终稿.xlsx"
data_sheet_name = 'data2'

# 创建所有点的Map
point_map = point_map.Map(data_path, data_sheet_name)

plt.figure(figsize=(10, 10))
x, y, z = point_map.point_list_numpy[:, 1], point_map.point_list_numpy[:, 2], point_map.point_list_numpy[:, 3]
ax = plt.gca(projection='3d')  # 创建一个三维的绘图工程
#  绘制校正点散点图
ax.scatter(x, y, z, c='y')  # 绘制数据点
# 设置坐标轴
ax.set_zlabel('Z')
ax.set_ylabel('Y')
ax.set_xlabel('X')

# plt.show()

# 创建起终点point对象
start_point = point.Point(point_map.point_list_numpy[0][0],
                          point_map.point_list_numpy[0][1],
                          point_map.point_list_numpy[0][2],
                          point_map.point_list_numpy[0][3],
                          point_map.point_list_numpy[0][4],
                          point_map.point_list_numpy[0][5])
end_index = len(point_map.point_list_numpy) - 1
end_point = point.Point(point_map.point_list_numpy[end_index][0],
                        point_map.point_list_numpy[end_index][1],
                        point_map.point_list_numpy[end_index][2],
                        point_map.point_list_numpy[end_index][3],
                        point_map.point_list_numpy[end_index][4],
                        point_map.point_list_numpy[end_index][5])

a_star = a_star.AStar(point_map, start_point, end_point)
a_star.RunAndSaveImage(ax, plt)
