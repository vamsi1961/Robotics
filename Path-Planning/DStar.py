import math
from sys import maxsize
import matplotlib.pyplot as plt

show_animation = True

class S:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.p = None
        self.s = "."
        self.t = "new"
        self.h = 0
        self.k = 0

    def c(self, s):
        if self.s == "#" or s.s == "#":
            return maxsize
        return math.sqrt((self.x - s.x) ** 2 + (self.y - s.y) ** 2)

    def set_s(self, s):
        if s not in ["s", ".", "#", "e", "*"]:
            return
        self.s = s

class M:
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.map = self.init_map()

    def init_map(self):
        m = []
        for i in range(self.row):
            t = []
            for j in range(self.col):
                t.append(S(i, j))
            m.append(t)
        return m

    def get_neighbors(self, s):
        s_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if s.x + i < 0 or s.x + i >= self.row:
                    continue
                if s.y + j < 0 or s.y + j >= self.col:
                    continue
                s_list.append(self.map[s.x + i][s.y + j])
        return s_list

    def set_obstacle(self, point_list):
        for x, y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue
            self.map[x][y].set_s("#")

class D:
    def __init__(self, m):
        self.map = m
        self.open_list = set()

    def process_s(self):
        x = self.min_s()
        if x is None:
            return -1
        k_old = self.get_kmin()
        self.remove(x)
        if k_old < x.h:
            for y in self.map.get_neighbors(x):
                if y.h <= k_old and x.h > y.h + x.c(y):
                    x.p = y
                    x.h = y.h + x.c(y)
        elif k_old == x.h:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or (y.p == x and y.h != x.h + x.c(y)) or (y.p != x and y.h > x.h + x.c(y)):
                    y.p = x
                    self.insert(y, x.h + x.c(y))
        else:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or (y.p == x and y.h != x.h + x.c(y)):
                    y.p = x
                    self.insert(y, x.h + x.c(y))
                else:
                    if y.p != x and y.h > x.h + x.c(y):
                        self.insert(y, x.h)
                    else:
                        if y.p != x and x.h > y.h + x.c(y) and y.t == "close" and y.h > k_old:
                            self.insert(y, y.h)
        return self.get_kmin()

    def min_s(self):
        if not self.open_list:
            return None
        min_s = min(self.open_list, key=lambda x: x.k)
        return min_s

    def get_kmin(self):
        if not self.open_list:
            return -1
        k_min = min(x.k for x in self.open_list)
        return k_min

    def insert(self, s, h_new):
        if s.t == "new":
            s.k = h_new
        elif s.t == "open":
            s.k = min(s.k, h_new)
        elif s.t == "close":
            s.k = min(s.h, h_new)
        s.h = h_new
        s.t = "open"
        self.open_list.add(s)

    def remove(self, s):
        if s.t == "open":
            s.t = "close"
        self.open_list.remove(s)

    def modify_c(self, x):
        if x.t == "close":
            self.insert(x, x.p.h + x.c(x.p))

    def run(self, start, end):
        rx, ry = [], []
        self.insert(end, 0.0)
        while True:
            self.process_s()
            if start.t == "close":
                break
        start.set_s("s")
        s = start
        s = s.p
        s.set_s("e")
        tmp = start
        while tmp != end:
            tmp.set_s("*")
            rx.append(tmp.x)
            ry.append(tmp.y)
            if show_animation:
                plt.plot(rx, ry, "-r")
                plt.pause(0.01)
            if tmp.p.s == "#":
                self.modify_c(tmp)
                continue
            tmp = tmp.p
        tmp.set_s("e")
        return rx, ry

    def modify(self, s):
        self.modify_c(s)
        while True:
            k_min = self.process_s()
            if k_min >= s.h:
                break

def main():
    m = M(100, 100)
    points = [(-10, -10), (-10, 60), (60, -10), (60, 60)]
    for i in range(-10, 40):
        points.append((20, i))
        points.append((40, 60 - i))
    m.set_obstacle(points)
    start = [10, 10]
    end = [50, 50]
    if show_animation:
        ox, oy = zip(*points)
        plt.plot(ox, oy, ".k")
        plt.plot(start[0], start[1], "og")
        plt.plot(end[0], end[1], "xb")
        plt.axis("equal")
    start = m.map[start[0]][start[1]]
    end = m.map[end[0]][end[1]]
    d = D(m)
    rx, ry = d.run(start, end)
    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()

if __name__ == '__main__':
    main()
