# point.py

import sys

class Point:
    def __init__(self, num, x, y, z, type, flag):
        self.num = num
        self.type = type
        self.flag = flag
        self.x = x
        self.y = y
        self.z = z
        self.cost = sys.maxsize
