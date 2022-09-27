import matplotlib.pyplot as plt
import math
import copy
from matplotlib.pyplot import MultipleLocator
import numpy as np
import sys

sys.setrecursionlimit(1500)


class Node:
    def __init__(self):
        self.father_index = [math.inf, math.inf]
        self.cost = math.inf

    def gatherAttrs(self):
        return ", ".join("{} = {}"
                         .format(k, getattr(self, k))
                         for k in self.__dict__.keys())

    def __str__(self):
        return "[{}]".format(self.gatherAttrs())


class PlotMap:
    def __init__(self, min_x, min_y, max_x, max_y, start_point, end_point):
        self.minX = min_x
        self.maxX = max_x
        self.minY = min_y
        self.maxY = max_y

        self.start = start_point
        self.end = end_point

        self.OpenList = dict()
        self.CloseList = dict()

        self.Open = []

        self.motion = [[1, 0, 1],
                       [0, 1, 1],
                       [-1, 0, 1],
                       [0, -1, 1],
                       [-1, -1, math.sqrt(2)],
                       [-1, 1, math.sqrt(2)],
                       [1, -1, math.sqrt(2)],
                       [1, 1, math.sqrt(2)]]

        self.border_point = []
        for ii in np.arange(min_x, max_x, 1):
            self.border_point.append([ii, min_y])

        for jj in np.arange(min_y, max_y, 1):
            self.border_point.append([min_x, jj])

        for ii in np.arange(min_x, max_x, 1):
            self.border_point.append([ii, max_y])
        #
        for jj in np.arange(min_y, max_y, 1):
            self.border_point.append([max_x, jj])

        self.border_point.append([max_x, max_y])

        for ij in np.arange(min_y, max_y - 10):
            self.border_point.append([-5, ij])

        for ij in np.arange(max_y - 20, max_y):
            self.border_point.append([5, ij])

        self.map_point = dict()
        index = 0
        init_node = Node()
        for ii in np.arange(min_x, max_x + 1, 1):
            for jj in np.arange(min_y, max_y + 1, 1):
                self.map_point[index] = copy.deepcopy(init_node)
                index = index + 1

    def Init(self):
        index = self.CalculateIndex(self.start[0], self.start[1])
        init_node = Node()
        init_node.father_index = [-1, -1]
        init_node.cost = 0
        self.map_point[index] = copy.deepcopy(init_node)

    def CalculateIndex(self, x, y):
        return (self.maxX - self.minX + 1) * (x - self.minX) + (y - self.minY)

    def Motion(self, xy):
        OpenList = []
        for id in xy:
            current_x = id[0]
            current_y = id[1]
            init_node = Node()
            for motion_x, motion_y, cost in self.motion:
                if (current_x + motion_x) == self.end[0] and (current_y + motion_y) == self.end[1]:
                    point_index_before = self.CalculateIndex(current_x, current_y)
                    print("-----: ", self.map_point[point_index_before].cost)
                    init_node.cost = cost + self.map_point[point_index_before].cost
                    init_node.father_index = [current_x, current_y]

                    point_index_current = self.CalculateIndex(current_x + motion_x, current_y + motion_y)
                    if self.map_point[point_index_current].cost > init_node.cost:
                        self.map_point[point_index_current] = copy.deepcopy(init_node)
                    return

                if self.Open.count([current_x + motion_x, current_y + motion_y]) == 0:
                    if self.border_point.count([current_x + motion_x, current_y + motion_y]) == 0:
                        self.Open.append([current_x + motion_x, current_y + motion_y])
                        OpenList.append([current_x + motion_x, current_y + motion_y])

                        point_index_before = self.CalculateIndex(current_x, current_y)
                        print("-----: ", self.map_point[point_index_before].cost)
                        init_node.cost = cost + self.map_point[point_index_before].cost
                        init_node.father_index = [current_x, current_y]

                        point_index_current = self.CalculateIndex(current_x + motion_x, current_y + motion_y)
                        if self.map_point[point_index_current].cost > init_node.cost:
                            self.map_point[point_index_current] = copy.deepcopy(init_node)

                        plt.scatter(current_x + motion_x, current_y + motion_y, c='g')
                        # if len(self.Open) % 5 == 0:
                        #     plt.pause(0.001)

                        # plt.pause(0.001)

        self.Motion(OpenList)


if __name__ == '__main__':
    startPoint = [-15, -15]
    endPoint = [15, 15]
    oPlotMap = PlotMap(-20, -20, 20, 20, startPoint, endPoint)
    oPlotMap.Init()
    plt.figure(figsize=(8, 8))
    q = []
    q.append(startPoint)
    for ij in oPlotMap.border_point:
        plt.scatter(ij[0], ij[1], c='k', marker="x")

    # x_major_locator = MultipleLocator(1)
    # y_major_locator = MultipleLocator(1)
    # ax = plt.gca()
    # ax.xaxis.set_major_locator(x_major_locator)
    plt.xlim(-30, 30)
    plt.ylim(-30, 30)
    # print(oPlotMap.CalculateIndex(-19, -20))
    plt.scatter(startPoint[0], startPoint[1], c='k', marker="x")
    plt.scatter(endPoint[0], endPoint[1], c='b', marker="x")
    oPlotMap.Motion(q)
    # for dd in oPlotMap.Open:
    #     plt.scatter(dd[0], dd[1], c='g')

    for key, value in oPlotMap.map_point.items():
        print("key: %s, value: %s", key, value)
    # plt.pause(0.001)
    print(oPlotMap.CalculateIndex(endPoint[0], endPoint[1]))
    plt.grid()
    plt.show()
