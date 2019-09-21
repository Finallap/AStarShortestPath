from scipy.io import loadmat
import numpy as np
import pandas as pd
import os

def dijkstra(graph, startIndex, path, cost, max):
    """
    求解各节点最短路径，获取path，和cost数组，
    startIndex: 起点索引
    path[i] 表示vi节点的前继节点索引，一直追溯到起点。
    cost[i] 表示vi节点的花费
    """
    lenth = len(graph)
    # print(lenth) # 矩阵的列数
    v = [0] * lenth
    # v表示结果集
    # 初始化 path，cost，V
    for i in range(lenth):
        if i == startIndex:
            v[startIndex] = 1
        else:
            cost[i] = graph[startIndex][i]
            path[i] = (startIndex if (cost[i] < max) else -1)
    print(v, cost, path)

    for i in range(1, lenth):
        minCost = max
        curNode = -1
        for w in range(lenth):
        # 找出未加入到v中的可连通点中，距离最近的点作为curNode
            if v[w] == 0 and cost[w] < minCost:
                minCost = cost[w]
                curNode = w

        if curNode == -1: break
        # 剩下都是不可通行的节点，跳出循环
        v[curNode] = 1
        # 加入到v中

        for w in range(lenth):
            if v[w] == 0 and (graph[curNode][w] + cost[curNode] < cost[w]):
                cost[w] = graph[curNode][w] + cost[curNode] # 更新权值
                path[w] = curNode # 更新路径
        # for 更新其他节点的权值（距离）和路径
    return path

if __name__ == '__main__':

    max = 2147483647
    # graph = [
    #     [max, max, 10, max, 30, 100],
    #     [max, max, 5, max, max, max],
    #     [max, 10, max, 50, max, max],
    #     [max, max, max, max, max, 10],
    #     [max, max, max, 20, max, 60],
    #     [max, max, max, max, max, max],
    #     ]
    mat_path = "G:/研究生课程相关/研究生数学建模/2019年中国研究生数学建模竞赛F题/pre_data.mat"
    mat_data = loadmat(mat_path)
    data = pd.DataFrame(mat_data['pre_data'])
    print(data)

    max = 2147483647
    point_number = 612

    # print(data.columns)
    distance_data = data[[0, 1, 7]]
    distance_data.columns = ['start', 'end', 'distance']
    # print(distance_data['distance'])

    a = np.zeros((point_number + 1, point_number + 1))
    for i in range(point_number + 1):
        for j in range(point_number + 1):
            a[i][j] = max
            for k in range(distance_data.shape[0]):
                if distance_data['start'][k] == i and distance_data['end'][k] == j:
                    a[i][j] = distance_data['distance'][k]
                    break
            print()
            print("%.4f%%" % ((i*612+j)/(612*612)))

    # print(a)

    # point_number = 612
    # a = np.zeros((point_number + 1, point_number + 1))
    # for i in range(point_number + 1):
    #     for j in range(point_number + 1):
    #
    #         a[i][j] = max
    #
    #
    # print(a)

    # length = len(graph)
    # path = [0] * length
    # cost = [0] * length
    # print(dijkstra(graph, 0, path, cost, max))