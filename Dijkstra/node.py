import math


class Node:
    def __init__(self):
        self.father_index = [math.inf, math.inf]
        self.cost = math.inf

    def get_self_attribute(self):
        return ", ".join("{} = {}"
                         .format(k, getattr(self, k))
                         for k in self.__dict__.keys())

    def __str__(self):
        return "[{}]".format(self.get_self_attribute())