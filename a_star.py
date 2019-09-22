# a_star.py
import sys
import time
import math
import point
import csv
import numpy as np


class AStar:
    def __init__(self, map, start_point, end_point):
        self.map = map
        self.open_set = []
        self.close_set = []
        self.start_point = start_point
        self.end_point = end_point

        # 数据集1的参数
        self.Alpha1 = 25
        self.Alpha2 = 15
        self.Beta1 = 20
        self.Beta2 = 25
        self.Delte = 0.001
        self.Theta = 30

        # 数据集2的参数
        # self.Alpha1 = 20
        # self.Alpha2 = 10
        # self.Beta1 = 15
        # self.Beta2 = 20
        # self.Delte = 0.001
        # self.Theta = 20

    def Distance(self, current, parent):
        x_dis = current.x - parent.x
        y_dis = current.y - parent.y
        z_dis = current.z - parent.z
        dis = math.sqrt(x_dis * x_dis + y_dis * y_dis + z_dis * z_dis)
        return dis

    def BaseCost(self, p):
        # Distance to start point
        return self.Distance(p, self.start_point)

    def HeuristicCost(self, p):
        return self.Distance(p, self.end_point)

    def TotalCost(self, p):
        # return self.HeuristicCost(p)
        return self.BaseCost(p) + self.HeuristicCost(p)

    def IsReachablePoint(self, current, parent):
        dis = self.Distance(current, parent)
        deviation = dis * self.Delte

        if current.type == 1 and deviation <= self.Alpha1 and deviation <= self.Alpha2:
            return True
        else:
            if current.type == 0 and deviation <= self.Beta1 and deviation <= self.Beta2:
                return True
            else:
                if current.num == self.end_point.num and deviation <= self.Theta and deviation <= self.Theta:
                    return True

        return False

    def IsSatisfyError(self, current, parent, horizontal, vertical):
        dis = self.Distance(current, parent)
        deviation = dis * self.Delte

        if current.type == 1 and vertical + deviation <= self.Alpha1 and horizontal + deviation <= self.Alpha2:
            return True
        elif current.type == 0 and vertical + deviation <= self.Beta1 and horizontal + deviation <= self.Beta2:
            return True
        elif current.type == 3 and vertical + deviation <= self.Theta and horizontal + deviation <= self.Theta:
            return True
        else:
            return False

    def IsInPointList(self, p, point_list):
        for point in point_list:
            if point.x == p.x and point.y == p.y and point.z == p.z:
                return True
        return False

    def IsInOpenList(self, p):
        return self.IsInPointList(p, self.open_set)

    def IsInCloseList(self, p):
        return self.IsInPointList(p, self.close_set)

    def IsStartPoint(self, p):
        return p.x == self.start_point.x and p.y == self.start_point.y and p.z == self.start_point.z

    def IsEndPoint(self, p):
        return p.x == self.end_point.x and p.y == self.end_point.y and p.z == self.end_point.z

    def SaveImage(self, plt):
        millis = int(round(time.time() * 1000))
        filename = './' + str(millis) + '.png'
        plt.savefig(filename)

    def ProcessPoint(self, current, parent, ax, plt):
        # CloseList中的点已经访问过，不再次进行访问，直接返回
        if self.IsInCloseList(current):
            return

        # 输出访问点的信息
        # print('Process Point [', current.num, ',', current.x, ',', current.y, ',', current.z, ']', ', cost: ',
        #       self.TotalCost(current))

        # OpenList中的点也不再次进行访问
        # 判断累计误差是否满足校验标准，满足才继续进一步处理
        if not self.IsInOpenList(current) and self.IsSatisfyError(current, parent, parent.horizontal, parent.vertical):
            # 计算目前选中的点和父节点将会发生的误差
            dis = self.Distance(current, parent)
            deviation = dis * self.Delte

            # 进行误差校正
            if current.type == 0:
                current.horizontal = 0
            else:
                current.horizontal = parent.horizontal + deviation

            if current.type == 1:
                current.vertical = 0
            else:
                current.vertical = parent.vertical + deviation

            current.distance = dis
            current.cumulative_distance = parent.cumulative_distance + dis
            current.parent = parent
            current.cost = self.TotalCost(current)
            self.open_set.append(current)  # 将目前选中的点加入open_set

            # 输出访问点的信息
            print('Process Point [', current.num, ']', ', cost: ', self.TotalCost(current),
                  ', Type: ', current.type,
                  ',current horizontal:', current.horizontal, ',current vertical:', current.vertical,
                  ',parent horizontal:', parent.horizontal, ',parent vertical:', parent.vertical)

            # 对查找过程进行画图，只输出结果时候可注释
            # x = [current.x, parent.x]
            # y = [current.y, parent.y]
            # z = [current.z, parent.z]
            # 将数组中的前两个点进行连线
            # ax.plot(x, y, z, c='r')
            # plt.draw()
            # self.SaveImage(plt)

    def SelectPointInOpenList(self):
        index = 0
        selected_index = -1
        min_cost = sys.maxsize
        for p in self.open_set:
            cost = self.TotalCost(p)
            if cost < min_cost:
                min_cost = cost
                selected_index = index
            index += 1
        return selected_index

    # 回溯输出最佳路径
    def BuildPath(self, p, ax, plt, start_time):
        data = np.array(['num', 'x', 'y', 'z', 'type', 'before horizontal',
                         'before vertical', 'after horizontal', 'after vertical', 'distance', 'cumulative distance'])

        path = []
        while True:
            path.insert(0, p)  # Insert first
            if self.IsStartPoint(p):
                break
            else:
                p = p.parent

        parent = self.start_point
        for p in path:
            # 画图
            x = [parent.x, p.x]
            y = [parent.y, p.y]
            z = [parent.z, p.z]
            # 将数组中的前两个点进行连线
            ax.plot(x, y, z, c='k', label='飞行路线')
            # 每画一个点便保存一张图片，不要可以注释
            # plt.draw()
            # self.SaveImage(plt)

            # 计算目前选中的点和父节点将会发生的误差
            dis = self.Distance(p, parent)
            deviation = dis * self.Delte

            # 输出最短路径信息
            print('Shortest Path Point [', p.num, ',', p.x, ',', p.y, ',', p.z, ']',
                  ', \ncost: ', p.cost, ', Type: ', p.type,
                  ',before horizontal:', parent.horizontal + deviation, ',before vertical:',
                  parent.vertical + deviation, '\n'
                                               ',current horizontal:', p.horizontal, ',current vertical:', p.vertical,
                  ',parent horizontal:', parent.horizontal, ',parent vertical:', parent.vertical, '\n')

            data = np.row_stack((data, [p.num, p.x, p.y, p.z, p.type, parent.horizontal + deviation,
                                        parent.vertical + deviation, p.horizontal, p.vertical, p.distance,
                                        p.cumulative_distance]))

            parent = p
        # 保存结果图片
        plt.draw()
        self.SaveImage(plt)

        with open('Record.csv', 'w', newline='') as t_file:
            csv_writer = csv.writer(t_file)
            for l in data:
                csv_writer.writerow(l)

        # 输出运行时间
        end_time = time.time()
        print('===== Algorithm finish in', int(end_time - start_time), ' seconds')

    # 查找能与目前的点可联通的点（只考虑两点间误差是否大于阈值，不考虑累计误差）
    def FindFeasiblePoint(self, current):
        feasible_point_list = []
        for i in range(len(self.map.point_list_numpy)):
            if current.num != self.map.point_list_numpy[i][0]:
                next_point = point.Point(self.map.point_list_numpy[i][0],
                                         self.map.point_list_numpy[i][1],
                                         self.map.point_list_numpy[i][2],
                                         self.map.point_list_numpy[i][3],
                                         self.map.point_list_numpy[i][4],
                                         self.map.point_list_numpy[i][5])
                if self.IsReachablePoint(next_point, current):
                    feasible_point_list.append(next_point)
        return feasible_point_list

    def RunAndSaveImage(self, ax, plt):
        start_time = time.time()

        # 初始化起点的cost，horizontal误差，vertical误差，并将其放入open_set
        self.start_point.cost = 0
        self.start_point.horizontal = 0
        self.start_point.vertical = 0
        self.start_point.distance = 0
        self.start_point.cumulative_distance = 0
        self.open_set.append(self.start_point)

        while True:
            # 选取open_set中，优先级最高（cost最小）的点进行尝试
            index = self.SelectPointInOpenList()

            # 如果open_set中已经不存在点，则路径寻找失败
            if index < 0:
                print('No path found, algorithm failed!!!')
                return
            p = self.open_set[index]

            # 判断是否抵达终点，如果到达抵达终点，则使用BuildPath回溯输出最佳路径
            if self.IsEndPoint(p):
                return self.BuildPath(p, ax, plt, start_time)

            # 没有抵达终点，则将这个点从open_set中删除，放入close_set
            del self.open_set[index]
            self.close_set.append(p)

            # 查找能与目前的点p可联通的点（只考虑两点间误差是否大于阈值，不考虑累计误差）
            feasible_point_list = self.FindFeasiblePoint(p)

            # 对可连通的点依次进行进一步判断尝试
            for next_point in feasible_point_list:
                self.ProcessPoint(next_point, p, ax, plt)
