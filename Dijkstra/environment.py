import numpy as np
import copy
from node import Node


class Environment:
    def __init__(self, min_x, min_y, max_x, max_y):
        self.minX = min_x
        self.maxX = max_x
        self.minY = min_y
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

        self.work_space = dict()
        init_node = Node()
        self.move_cost = dict()

        for ii in np.arange(min_x, max_x + 1, 1):
            for jj in np.arange(min_y, max_y + 1, 1):
                self.work_space[self.calculate_index(ii, jj)] = [ii, jj]
                self.move_cost[self.calculate_index(ii, jj)] = copy.deepcopy(init_node)

    def get_workspace(self):
        return self.work_space

    def get_border(self):
        return self.border_point

    def calculate_index(self, x, y):
        return (self.maxX - self.minX + 1) * (x - self.minX) + (y - self.minY)

    def get_init_motion_cost(self):
        return self.move_cost


