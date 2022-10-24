import matplotlib.pyplot as plt
import math
import copy
import numpy as np


class Node:
    def __init__(self):
        self.father_index = [math.inf, math.inf]
        self.currrent_index = [math.inf, math.inf]
        self.cost = math.inf

    def gatherAttrs(self):
        return ", ".join("{} = {}"
                         .format(k, getattr(self, k))
                         for k in self.__dict__.keys())

    def __str__(self):
        return "[{}]".format(self.gatherAttrs())


class RRTGridMap:
    def __init__(self, min_x, min_y, max_x, max_y, start_point, end_point, step, tolerance):
        self.minX = min_x
        self.maxX = max_x
        self.minY = min_y
        self.maxY = max_y
        self.step = step
        self.tolerance = tolerance
        self.path = dict()

        self.start = start_point
        self.end = end_point
        self.Index = dict()
        self.CloseList = dict()
        self.Open = dict()
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

    def BuildingBoundaryMap(self):
        for ij in self.border_point:
            plt.scatter(ij[0], ij[1], c='k', marker="x")
        plt.scatter(self.start[0], self.start[1], c='r', marker="o")
        plt.scatter(self.end[0], self.end[1], c='r', marker="o")

    def RRTPlanning(self):
        init_node = Node()
        index = self.CalculateIndex(self.start[0], self.start[1])
        self.CloseList[index] = [self.start[0], self.start[1]]
        while True:
            point_x, point_y = self.GeneratePoint()
            pose = self.GetMinDistance(point_x, point_y)
            init_node.father_index = [pose[0], pose[1]]
            A, B, C = self.GeneralEquation(pose[0], pose[1], point_x, point_y)
            if self.IsOkPoint(A, B, C, pose, point_x, point_y):
                if math.hypot(point_x - self.end[0], point_y - self.end[1]) < self.tolerance:
                    init_node.currrent_index = [self.end[0], self.end[1]]
                    self.Open[self.CalculateIndex(self.end[0], self.end[1])] = (copy.deepcopy(init_node))
                    break
                init_node.currrent_index = [point_x, point_y]
                self.CloseList[self.CalculateIndex(point_x, point_y)] = [point_x, point_y]
                self.Open[self.CalculateIndex(point_x, point_y)] = copy.deepcopy(init_node)
        self.PlotMap()
        return

    def GetMinDistance(self, point_x, point_y):
        distance = dict()
        for value in self.CloseList.values():
            cost = math.hypot(point_x - value[0], point_y - value[1])
            index = self.CalculateIndex(value[0], value[1])
            distance[index] = [cost]
        index = min(distance, key=lambda k: distance[k])
        return self.Index[index]

    def CalculateIndex(self, x, y):
        return (self.maxX - self.minX + 1) * (x - self.minX) + (y - self.minY)

    def GeneralEquation(self, first_x, first_y, second_x, second_y):
        # 一般式 Ax+By+C=0
        A = second_y - first_y
        B = first_x - second_x
        C = second_x * first_y - first_x * second_y
        return A, B, C

    def GeneratePoint(self):
        while True:
            point_x = round(np.random.uniform(self.minX, self.maxX))
            point_y = round(np.random.uniform(self.minY, self.maxY))
            index = self.CalculateIndex(point_x, point_y)
            if self.CloseList.get(index) is None:
                return point_x, point_y

    def IsOkPoint(self, A, B, C, pose, point_x, point_y):
        if B == 0:
            if point_y > pose[1]:
                for num in range(pose[1], point_y + 1):
                    for ite in self.border_point:
                        distance = math.hypot(point_x - ite[0], num - ite[1])
                        if distance <= self.tolerance:
                            return False
            else:
                for num in range(point_y, pose[1] + 1):
                    for ite in self.border_point:
                        distance = math.hypot(point_x - ite[0], num - ite[1])
                        if distance <= self.tolerance:
                            return False

        else:
            if point_x > pose[0]:
                for num in range(pose[0], point_x + 1):
                    y = (-A * num - C) / B
                    for ite in self.border_point:
                        distance = math.hypot(num - ite[0], y - ite[1])
                        if distance <= self.tolerance:
                            return False
            else:
                for num in range(point_x, pose[0] + 1):
                    y = (-A * num - C) / B
                    for ite in self.border_point:
                        distance = math.hypot(num - ite[0], y - ite[1])
                        if distance <= self.tolerance:
                            return False
        return True

    def FindPath(self):
        xx = []
        yy = []
        xx.append(self.end[0])
        yy.append(self.end[1])
        index = self.CalculateIndex(self.end[0], self.end[1])
        value = self.Open[index]
        print("value:  ", value.father_index)
        xx.append(value.father_index[0])
        yy.append(value.father_index[1])
        while True:
            if value.father_index == [self.start[0], self.start[1]]:
                xx.append(self.start[0])
                yy.append(self.start[1])
                xx.reverse()
                yy.reverse()
                return xx, yy
            else:
                index = self.CalculateIndex(value.father_index[0], value.father_index[1])
                value = self.Open[index]
                print("value:  ", value.father_index)
                xx.append(value.father_index[0])
                yy.append(value.father_index[1])

    def PlotMap(self):
        plt.figure(figsize=(18, 10))
        plt.xlim(self.minX - 1, self.maxX + 1)
        plt.ylim(self.minY - 1, self.maxY + 1)
        plt.grid()
        plt.xticks(np.arange(self.minX - 1, self.maxX + 1,1))
        plt.yticks(np.arange(self.minY - 1, self.maxY + 1,1))
        self.BuildingBoundaryMap()
        for ii in self.Open.values():
            xx = [ii.father_index[0], ii.currrent_index[0]]
            yy = [ii.father_index[1], ii.currrent_index[1]]
            plt.plot(xx, yy, c="g", marker="o")
            plt.pause(0.001)
        x1, y1 = self.FindPath()
        plt.plot(x1, y1, c='r', marker="*")
        plt.pause(0.001)


if __name__ == '__main__':
    startPoint = [-15, -15]
    endPoint = [15, 15]
    step = 1
    tolerance = 2
    Test = RRTGridMap(-20, -20, 20, 20, startPoint, endPoint, step, tolerance)
    Test.RRTPlanning()
    plt.pause(0.001)

    plt.show()
