import matplotlib.pyplot as plt
import math
import copy
import numpy as np
import sys

sys.setrecursionlimit(1500)


class Node:
    def __init__(self):
        self.father_index = [math.inf, math.inf]
        self.H_cost = math.inf
        self.G_cost = math.inf
        self.F_cost = math.inf

    def gatherAttrs(self):
        return ", ".join("{} = {}"
                         .format(k, getattr(self, k))
                         for k in self.__dict__.keys())

    def __str__(self):
        return "[{}]".format(self.gatherAttrs())


class PlotMap:
    def __init__(self, min_x, min_y, max_x, max_y, start_point, end_point, move_wight):
        # move_wight [0 - 1]
        self.minX = min_x
        self.maxX = max_x
        self.minY = min_y
        self.maxY = max_y

        self.start = start_point
        self.end = end_point
        self.Index = dict()
        self.CloseList = dict()
        self.Open = []

        self.motion = [[1, 0, move_wight],
                       [0, 1, move_wight],
                       [-1, 0, move_wight],
                       [0, -1, move_wight],
                       [-1, -1, math.hypot(move_wight, move_wight)],
                       [-1, 1, math.hypot(move_wight, move_wight)],
                       [1, -1, math.hypot(move_wight, move_wight)],
                       [1, 1, math.hypot(move_wight, move_wight)]]

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
                self.Index[index] = [ii, jj]
                index = index + 1

    def CalculateIndex(self, x, y):
        return (self.maxX - self.minX + 1) * (x - self.minX) + (y - self.minY)

    def GetMinNode(self, dirctory):
        key_index = math.inf
        index = min(dirctory, key=lambda k: dirctory[k].F_cost)
        for key, value in dirctory.items():
            # print("+++++++++++: ", value.F_cost)
            if value.F_cost == dirctory[index].F_cost:
                if key_index == math.inf:
                    key_index = key
                    # print("------")
                elif value.H_cost < dirctory[index].H_cost:
                    key_index = key
                    # print("=======")
                elif value.H_cost == dirctory[index].H_cost:
                    if value.G_cost < dirctory[index].G_cost:
                        key_index = key
        return key_index

    def Dijkstra(self):
        plt.figure(figsize=(8, 8))
        self.BuildingBoundaryMap()
        index = self.CalculateIndex(self.start[0], self.start[1])
        init_node = Node()
        init_node.father_index = [-1, -1]
        init_node.G_cost = 0
        init_node.H_cost = init_node.H_cost = math.hypot(self.start[0] - self.end[0], self.start[1] - self.end[1])
        # init_node.H_cost = abs(self.start[0] - self.end[0]) + abs(self.start[1] - self.end[1])
        init_node.F_cost = init_node.H_cost
        self.map_point[index] = copy.deepcopy(init_node)
        self.CloseList[index] = copy.deepcopy(init_node)
        current_x = self.start[0]
        current_y = self.start[1]
        self.map_point.pop(index)
        for motion_x, motion_y, cost in self.motion:
            target_x = current_x + motion_x
            target_y = current_y + motion_y

            if target_x == self.end[0] and target_y == self.end[1]:
                break
            point_index_before = self.CalculateIndex(current_x, current_y)
            init_node.G_cost = cost + self.CloseList[point_index_before].G_cost
            init_node.H_cost = math.hypot(target_x - self.end[0], target_y - self.end[1])
            # init_node.H_cost = abs(target_x - self.end[0]) + abs(target_y - self.end[1])
            init_node.F_cost = init_node.H_cost + init_node.G_cost

            init_node.father_index = [current_x, current_y]

            point_index_current = self.CalculateIndex(target_x, target_y)
            if self.map_point[point_index_current].F_cost > init_node.F_cost:
                self.map_point[point_index_current] = copy.deepcopy(init_node)
        while 1:
            min_key = self.GetMinNode(self.map_point)
            current_x = self.Index[min_key][0]
            current_y = self.Index[min_key][1]
            self.Open.append([current_x, current_y])
            self.CloseList[min_key] = self.map_point[min_key]
            self.map_point.pop(min_key)
            if current_x == self.end[0] and current_y == self.end[1]:
                break
            for motion_x, motion_y, cost in self.motion:
                target_x = current_x + motion_x
                target_y = current_y + motion_y
                if self.border_point.count([target_x, target_y]) == 0:
                    point_index_current = self.CalculateIndex(target_x, target_y)
                    if self.CloseList.get(point_index_current):
                        continue
                    else:
                        point_index_before = self.CalculateIndex(current_x, current_y)
                        init_node.G_cost = cost + cost + self.CloseList[point_index_before].G_cost
                        init_node.H_cost = math.hypot(target_x - self.end[0], target_y - self.end[1])
                        # init_node.H_cost = abs(target_x - self.end[0]) + abs(target_y - self.end[1])
                        init_node.F_cost = init_node.H_cost + init_node.G_cost
                        init_node.father_index = [current_x, current_y]
                        if self.map_point[point_index_current].F_cost >= init_node.F_cost:
                            self.map_point[point_index_current] = copy.deepcopy(init_node)

    def GetPath(self):
        iCount = 0
        for ii in self.Open:
            plt.plot(ii[0], ii[1], c='g', marker='o')
            iCount = iCount + 1
            if iCount % 5 == 0:
                plt.pause(0.1)
        xy = [endPoint]
        EndIndex = self.CalculateIndex(endPoint[0], endPoint[1])
        xy.append(self.CloseList[EndIndex].father_index)
        cc = self.CloseList[EndIndex].father_index
        xy.append(cc)
        while 1:
            EndIndex = self.CalculateIndex(cc[0], cc[1])
            cc = self.CloseList[EndIndex].father_index
            xy.append(cc)
            if self.start == cc:
                return xy

    def BuildingBoundaryMap(self):
        for ij in self.border_point:
            plt.scatter(ij[0], ij[1], c='k', marker="x")
        plt.scatter(self.start[0], self.start[1], c='b', marker="o")
        plt.scatter(self.end[0], self.end[1], c='b', marker="o")

    def PlotCalculationProcess(self):
        ll = self.GetPath()
        rx = []
        ry = []
        for ResolutionWay in ll:
            rx.append(ResolutionWay[0])
            ry.append(ResolutionWay[1])
        plt.plot(rx, ry, c='r', marker='*')
        plt.pause(1)
        plt.grid()
        plt.show()


if __name__ == '__main__':
    MoveWeight = 0.1
    startPoint = [-15, -15]
    endPoint = [15, 15]
    oPlotMap = PlotMap(-20, -20, 20, 20, startPoint, endPoint, MoveWeight)
    Start = [startPoint]
    oPlotMap.Dijkstra()
    oPlotMap.PlotCalculationProcess()
