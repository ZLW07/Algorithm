import matplotlib.pyplot as plt
import math
import copy
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
        self.CloseList = []

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

        for ij in np.arange(max_y - 15, max_y):
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

    def Dijkstra(self, xy):
        for PointPosition in xy:
            current_x = PointPosition[0]
            current_y = PointPosition[1]
            init_node = Node()
            for motion_x, motion_y, cost in self.motion:
                target_x = current_x + motion_x
                target_y = current_y + motion_y
                if target_x == self.end[0] and target_y == self.end[1]:
                    point_index_before = self.CalculateIndex(current_x, current_y)
                    init_node.cost = cost + self.map_point[point_index_before].cost
                    init_node.father_index = [current_x, current_y]
                    point_index_current = self.CalculateIndex(target_x, target_y)
                    if self.map_point[point_index_current].cost > init_node.cost:
                        self.map_point[point_index_current] = copy.deepcopy(init_node)
                    return

                if self.Open.count([target_x, target_y]) == 0:
                    if self.border_point.count([target_x, target_y]) == 0:
                        if target_x != self.start[0] or target_y != self.start[1]:
                            self.Open.append([target_x, target_y])
                        point_index_before = self.CalculateIndex(current_x, current_y)
                        init_node.cost = cost + self.map_point[point_index_before].cost
                        init_node.father_index = [current_x, current_y]
                        point_index_current = self.CalculateIndex(target_x, target_y)
                        if self.map_point[point_index_current].cost > init_node.cost:
                            self.map_point[point_index_current] = copy.deepcopy(init_node)
        self.Dijkstra(self.Open)

    def GetPath(self):
        EndIndex = self.CalculateIndex(endPoint[0], endPoint[1])
        self.CloseList.append([endPoint[0], endPoint[1]])
        dd = self.map_point[EndIndex].father_index
        self.CloseList.insert(0, dd)
        index = self.CalculateIndex(dd[0], dd[1])
        while 1:
            cc = self.map_point[index].father_index
            self.CloseList.insert(0, cc)
            if cc[0] == self.start[0] and cc[1] == self.start[1]:
                return
            index = self.CalculateIndex(cc[0], cc[1])

    def BuildingBoundaryMap(self):
        for ij in self.border_point:
            plt.scatter(ij[0], ij[1], c='k', marker="x")
        plt.scatter(self.start[0], self.start[1], c='b', marker="o")
        plt.scatter(self.end[0], self.end[1], c='b', marker="o")

    def PlotCalculationProcess(self):
        plt.figure(figsize=(8, 8))
        self.BuildingBoundaryMap()
        iCount = 0
        for Position in self.Open:
            iCount = iCount + 1
            if iCount % 30 == 0:
                plt.pause(0.01)
            plt.plot(Position[0], Position[1], c='g', marker='o')

        self.GetPath()
        rx = []
        ry = []
        for ResolutionWay in self.CloseList:
            rx.append(ResolutionWay[0])
            ry.append(ResolutionWay[1])
        plt.plot(rx, ry, c='r', marker='*')
        plt.pause(1)
        plt.grid()
        plt.show()


if __name__ == '__main__':
    startPoint = [-15, -15]
    endPoint = [15, 15]
    oPlotMap = PlotMap(-20, -20, 20, 20, startPoint, endPoint)
    oPlotMap.Init()
    Start = [startPoint]
    oPlotMap.Dijkstra(Start)
    oPlotMap.PlotCalculationProcess()
