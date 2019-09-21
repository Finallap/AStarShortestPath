# a_star.py
import sys
import time
import math
import point


class AStar:
    def __init__(self, map, start_point, end_point):
        self.map = map
        self.open_set = []
        self.close_set = []
        self.start_point = start_point
        self.end_point = end_point
        # self.Alpha1 = 25
        # self.Alpha2 = 15
        # self.Beta1 = 20
        # self.Beta2 = 25
        # self.Delte = 0.001
        # self.Theta = 30
        self.Alpha1 = 20
        self.Alpha2 = 10
        self.Beta1 = 15
        self.Beta2 = 20
        self.Delte = 0.001
        self.Theta = 20

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

        if current.type == 1 and horizontal + deviation <= self.Alpha1 and vertical + deviation <= self.Alpha2:
            return True
        elif current.type == 0 and horizontal + deviation <= self.Beta1 and vertical + deviation <= self.Beta2:
            return True
        elif current.type == 3 and horizontal + deviation <= self.Theta and vertical + deviation <= self.Theta:
            return True
        else:
            return False

    def IsValidPoint(self, current, parent):
        if current.x < 0 or current.y < 0 or current.z < 0:
            return False
        if self.IsReachablePoint(current, parent):
            return True
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
        if not self.IsValidPoint(current, parent):
            return  # Do nothing for invalid point
        if self.IsInCloseList(current):
            return  # Do nothing for visited point
        # print('Process Point [', current.num, ',', current.x, ',', current.y, ',', current.z, ']', ', cost: ',
        #       self.TotalCost(current))
        # todo 校验的约束条件在这里
        if not self.IsInOpenList(current) and self.IsSatisfyError(current, parent, parent.horizontal, parent.vertical):
            # if not self.IsInOpenList(current):
            dis = self.Distance(current, parent)
            deviation = dis * self.Delte

            # 进行误差校正
            if current.type == 0:
                current.vertical = 0
            else:
                current.vertical = parent.vertical + deviation

            if current.type == 1:
                current.horizontal = 0
            else:
                current.horizontal = parent.horizontal + deviation

            current.parent = parent
            current.cost = self.TotalCost(current)
            self.open_set.append(current)
            print('Process Point [', current.num, ']', ', cost: ',
                  self.TotalCost(current), ',current horizontal:', current.horizontal, ',current vertical:',
                  current.vertical,
                  ',parent horizontal:', parent.horizontal, ',parent vertical:', parent.vertical)
            # 画图
            x = [current.x, parent.x]
            y = [current.y, parent.y]
            z = [current.z, parent.z]
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

    # todo BuildPath
    def BuildPath(self, p, ax, plt, start_time):
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
            ax.plot(x, y, z, c='k')
            plt.draw()
            self.SaveImage(plt)
            print('Shortest Path Point [', p.num, ',', p.x, ',', p.y, ',', p.z, ']', ', \ncost: ', p.cost,
                  ',current horizontal:', p.horizontal, ',current vertical:', p.vertical,
                  ',parent horizontal:', parent.horizontal, ',parent vertical:', parent.vertical, '\n')
            parent = p
        end_time = time.time()
        print('===== Algorithm finish in', int(end_time - start_time), ' seconds')

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

        self.start_point.cost = 0
        self.start_point.horizontal = 0
        self.start_point.vertical = 0
        self.open_set.append(self.start_point)

        while True:
            index = self.SelectPointInOpenList()
            if index < 0:
                print('No path found, algorithm failed!!!')
                return
            p = self.open_set[index]

            if self.IsEndPoint(p):
                return self.BuildPath(p, ax, plt, start_time)

            del self.open_set[index]
            self.close_set.append(p)

            # 找最基础可行的点
            feasible_point_list = self.FindFeasiblePoint(p)

            for next_point in feasible_point_list:
                self.ProcessPoint(next_point, p, ax, plt)
