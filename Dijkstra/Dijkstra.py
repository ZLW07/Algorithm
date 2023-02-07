import matplotlib.pyplot as plt
import math
import copy
import numpy as np
import sys
from pathlib import Path

from environment import Environment
from node import Node

sys.path.append(str(Path(__file__).resolve().parents[1]))


sys.setrecursionlimit(1500)


class Dijkstra:
    def __init__(self, min_x, min_y, max_x, max_y, start_point, end_point):
        self.env = Environment(min_x, min_y, max_x, max_y)
        self.border_point = self.env.get_border()
        self.work_space = self.env.get_workspace()
        self.move_cost = self.env.get_init_motion_cost()
        self.start = start_point
        self.end = end_point
        self.CloseList = dict()
        self.searched_space = []
        self.motion = [[1, 0, 1],
                       [0, 1, 1],
                       [-1, 0, 1],
                       [0, -1, 1],
                       [-1, -1, math.sqrt(2)],
                       [-1, 1, math.sqrt(2)],
                       [1, -1, math.sqrt(2)],
                       [1, 1, math.sqrt(2)]]

    def CalculateIndex(self, x, y):
        return self.env.calculate_index(x, y)

    def Dijkstra(self):
        plt.figure(figsize=(8, 8))
        self.BuildingBoundaryMap()
        index = self.CalculateIndex(self.start[0], self.start[1])
        init_node = Node()
        init_node.father_index = [-1, -1]
        init_node.cost = 0
        self.move_cost[index] = copy.deepcopy(init_node)
        self.CloseList[index] = copy.deepcopy(init_node)
        current_x = self.start[0]
        current_y = self.start[1]
        self.move_cost.pop(index)
        for motion_x, motion_y, cost in self.motion:
            target_x = current_x + motion_x
            target_y = current_y + motion_y

            if target_x == self.end[0] and target_y == self.end[1]:
                break
            point_index_before = self.CalculateIndex(current_x, current_y)
            init_node.cost = cost + self.CloseList[point_index_before].cost
            init_node.father_index = [current_x, current_y]
            point_index_current = self.CalculateIndex(target_x, target_y)
            if self.move_cost[point_index_current].cost > init_node.cost:
                self.move_cost[point_index_current] = copy.deepcopy(init_node)
        while 1:
            min_key = min(self.move_cost, key=lambda k: self.move_cost[k].cost)
            current_x = self.work_space[min_key][0]
            current_y = self.work_space[min_key][1]
            self.searched_space.append([current_x, current_y])
            self.CloseList[min_key] = self.move_cost[min_key]
            self.move_cost.pop(min_key)
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
                        init_node.cost = cost + self.CloseList[point_index_before].cost
                        init_node.father_index = [current_x, current_y]
                        if self.move_cost[point_index_current].cost >= init_node.cost:
                            self.move_cost[point_index_current] = copy.deepcopy(init_node)

    def GetPath(self):
        iCount = 0
        for ii in self.searched_space:
            plt.plot(ii[0], ii[1], c='g', marker='o')
            iCount = iCount + 1
            if iCount % 30 == 0:
                plt.pause(0.001)
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
    startPoint = [-15, -15]
    endPoint = [15, 15]
    oPlotMap = Dijkstra(-20, -20, 20, 20, startPoint, endPoint)
    Start = [startPoint]
    oPlotMap.Dijkstra()
    oPlotMap.PlotCalculationProcess()
