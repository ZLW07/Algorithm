import math
import copy


class Node:
    def __init__(self):
        self.father_index = -1
        self.cost = math.inf

    def gatherAttrs(self):
        return ", ".join("{} = {}"
                         .format(k, getattr(self, k))
                         for k in self.__dict__.keys())

    def __str__(self):
        return "[{}]".format(self.gatherAttrs())


class TestDijstra:

    def __init__(self, start_point, end_point):
        self.Map = []
        self.Map.append([0, 1, 4])
        self.Map.append([0, 7, 8])
        self.Map.append([1, 7, 11])
        self.Map.append([1, 2, 8])
        self.Map.append([7, 8, 7])
        self.Map.append([7, 6, 1])
        self.Map.append([2, 8, 2])
        self.Map.append([2, 5, 4])
        self.Map.append([2, 3, 7])
        self.Map.append([8, 6, 6])
        self.Map.append([6, 5, 2])
        self.Map.append([3, 5, 14])
        self.Map.append([3, 4, 9])
        self.Map.append([5, 4, 10])

        self.start_point = start_point
        self.end_point = end_point

        self.best_way = []

        self.open_list = dict()
        self.close_list = dict()
        init_node = Node()

        self.close_list[0] = copy.deepcopy(init_node)
        self.close_list[1] = copy.deepcopy(init_node)
        self.close_list[2] = copy.deepcopy(init_node)
        self.close_list[3] = copy.deepcopy(init_node)
        self.close_list[4] = copy.deepcopy(init_node)
        self.close_list[5] = copy.deepcopy(init_node)
        self.close_list[6] = copy.deepcopy(init_node)
        self.close_list[7] = copy.deepcopy(init_node)
        self.close_list[8] = copy.deepcopy(init_node)

    def PrintMap(self):
        for ii in self.Map:
            print("----map node and cost is ", ii)

    def FindMap(self):
        self.close_list[self.start_point].cost = 0
        temp_node = Node()
        temp_node.father_index = -1
        temp_node.cost = 0
        self.open_list[self.start_point] = copy.deepcopy(temp_node)
        for ii in self.Map:
            if self.start_point == ii[0]:
                temp_node.father_index = ii[0]
                temp_node.cost = ii[2]
                self.close_list[ii[1]] = copy.deepcopy(temp_node)
            elif self.start_point == ii[1]:
                temp_node.father_index = ii[1]
                temp_node.cost = ii[2]
                self.close_list[ii[1]] = copy.deepcopy(temp_node)
        while 1:
            min_cost = math.inf
            min_key = 0
            for key, value in self.close_list.items():
                if key not in self.open_list:
                    if min_cost > value.cost:
                        min_cost = value.cost
                        min_key = key
            temp_node.father_index = self.close_list[min_key].father_index
            temp_node.cost = self.close_list[min_key].cost
            self.open_list[min_key] = copy.deepcopy(temp_node)

            if min_key == self.end_point:
                break

            for ii in self.Map:
                if min_key == ii[0]:
                    if ii[1] not in self.open_list:
                        temp_node.father_index = ii[0]
                        temp_node.cost = ii[2] + self.close_list[min_key].cost
                        if self.close_list[ii[1]].cost > temp_node.cost:
                            self.close_list[ii[1]] = copy.deepcopy(temp_node)
                elif min_key == ii[1]:
                    if ii[0] not in self.open_list:
                        temp_node.father_index = ii[1]
                        temp_node.cost = ii[2] + self.close_list[min_key].cost
                        if self.close_list[ii[0]].cost > temp_node.cost:
                            self.close_list[ii[0]] = copy.deepcopy(temp_node)

        for key, value in self.open_list.items():
            print("open_list: ", key, " : ", value)

    def GetPath(self):
        path = []
        index = self.end_point
        while True:
            path.append(index)
            if index == self.start_point:
                break
            index = self.open_list[index].father_index
        path.reverse()
        print(path)


if __name__ == '__main__':
    start = 0
    end = 4
    Otest = TestDijstra(start, end)
    Otest.FindMap()
    Otest.GetPath()
