import matplotlib.pyplot as plt
import Queue
import math

from IPython.utils.tests.test_wildcard import q

show_animation = True


class Node:

    def __init__(self, x, y, value):
        self.x = x
        self.y = y
        self.val = value
        self.next_x = -1
        self.next_y = -1
        self.prev = [[-1, -1, -1, -1, -1, -1, -1, -1],
                     [-1, -1, -1, -1, -1, -1, -1, -1]]

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.val) + "," + str(self.next_x) + "," + str(self.next_y)


class NodeMap:

    def __init__(self, min_x, max_x, min_y, max_y, world_gx, world_gy, reso):
        self.map = dict()
        self.real_min_x = min_x
        self.real_max_x = max_x
        self.real_min_y = min_y
        self.real_max_y = max_y
        self.reso = reso
        self.size_x = calc_map_size(max_x, min_x, reso)
        self.size_y = calc_map_size(max_y, min_y, reso)
        self.gx, self.gy = world_to_map(self, world_gx, world_gy)


def calc_map_size(max_val, min_val, resolution):
    return int(round((max_val - min_val) / resolution))


def world_to_map(node_map, point_x, point_y):
    map_point_x = (point_x - node_map.real_min_x) / node_map.reso
    map_point_y = (point_y - node_map.real_min_y) / node_map.reso
    return map_point_x, map_point_y


def map_to_world(node_map, point_x, point_y):
    world_point_x = (point_x + node_map.real_min_x) / node_map.reso
    world_point_y = (point_y + node_map.real_min_y) / node_map.reso
    return world_point_x, world_point_y


def construct_node_map(min_x, max_x, min_y, max_y, world_gx, world_gy, resolution):
    node_map = NodeMap(min_x, max_x, min_y, max_y, world_gx, world_gy, resolution)
    size_x = node_map.size_x
    size_y = node_map.size_y
    gx = node_map.gx
    gy = node_map.gy
    for i in range(size_x):
        for j in range(size_y):
            if i == 0 or j == 0 or i == (size_x-1) or j == (size_y-1):
                node_map.map[i, j] = Node(i, j, 1.0)
            elif i == gx and j == gy:
                node_map.map[i, j] = Node(i, j, 2.0)
            else:
                node_map.map[i, j] = Node(i, j, 0.0)

    print_node_map(node_map, 2, 2)
    return node_map, gx, gy


def print_node_map(node_map, robot_x, robot_y):
    map_node = node_map.map
    size_x = node_map.size_x
    size_y = node_map.size_y
    for i in range(size_x):
        for j in range(size_y):
            if map_node[i, j].val == 1:
                plt.plot(i,  j, ".k")
            elif map_node[i, j].val == 2:
                plt.plot(i, j, "xb")
            elif i == robot_x and j == robot_y:
                plt.plot(i, j, "or")

    plt.grid(True)
    plt.show()


def wave_front(node_map):
    # initilaze
    q = Queue.Queue()
    start = Node(0, 0, -2.0)
    finish = Node(0, 0, -3.0)
    q.put(start)
    q.put(node_map.map[node_map.gx, node_map.gy])
    q.put(finish)
    d = 2       # rank (value) of node
    flag_start = 0;
    # itaration
    while True:
        temp_node = q.get()
        if temp_node.val == -2.0:  # start node
            flag_start = 1
            q.put(node_map.map[temp_node.x, temp_node.y])
        elif temp_node.val == -3.0:
            if flag_start == 1:
                break
            else:
                q.put(node_map.map[temp_node.x, temp_node.y])
        else:
            flag_start = 0
            node_map.map[temp_node.x, temp_node.y].val = d
            add_connected(node_map, q, temp_node)
    return node_map


def add_connected(node_map, queue, node):
     for i in [node.x - 1, node.x, node.x + 1]:
         for j in [node.y - 1, node.y, node.y + 1]:
             if i >= 0 and j >=0 and i < node_map.size_x and j < node_map.size_y:  # in the map
                if node_map.map[i, j].val == 0:  # first time in queue
                    node_map.map[i, j].val = -1
                    queue.put(node_map.map[i, j])





if __name__ == '__main__':
    min_x = 0.0     # [cm]
    max_x = 50.0    # [cm]
    min_y = -25.0   # [cm]
    max_y = 25.0    # [cm]
    real_gx = 40.0  # [cm]
    real_gy = 15.0  # [cm]
    reso = 1        # [cm]
    node_map = construct_node_map(min_x, max_x, min_y, max_y, real_gx, real_gy, reso)
