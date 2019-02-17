import matplotlib.pyplot as plt
import numpy as np
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
        self.prev = Queue.PriorityQueue()

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


class Cord:
    def __init__(self, priority, loc_x, loc_y):
        self.priority = priority
        self.x = loc_x
        self.y = loc_y
        return

    def __cmp__(self, other):
        return cmp(self.priority, other.priority)

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.priority)


def calc_map_size(max_val, min_val, resolution):
    return int(round((max_val - min_val) / resolution)) + 2


def world_to_map(node_map, point_x, point_y):
    map_point_x = int(round((point_x - node_map.real_min_x) / node_map.reso)) + 1
    map_point_y = int(round((point_y - node_map.real_min_y) / node_map.reso)) + 1
    return map_point_x, map_point_y


def map_to_world(node_map, point_x, point_y):
    world_point_x = (point_x - 1) * node_map.reso + node_map.real_min_x
    world_point_y = (point_y - 1) * node_map.reso + node_map.real_min_y
    return world_point_x, world_point_y


def construct_node_map(node_map):
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
    # print_node_map(node_map, 2, 2)
    # print_node_mat(node_map, 2, 2)


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
    return plt


def print_node_mat(node_map, robot_x, robot_y):
    fig, ax = plt.subplots()
    intersection_matrix = np.random.randint(0, 1, size=(node_map.size_x, node_map.size_y))
    for i in xrange(node_map.size_x):
        for j in xrange(node_map.size_y):
            intersection_matrix[i, j] = node_map.map[i, j].val
    ax.matshow(np.transpose(intersection_matrix), cmap=plt.cm.Blues)
    for i in xrange(node_map.size_x):
        for j in xrange(node_map.size_y):
            c = intersection_matrix[i, j]
            ax.text(i, j, str(c), va='center', ha='center')
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
            q.put(start)
        elif temp_node.val == -3.0:  # finish node
            if flag_start == 1:
                break
            else:
                d += 1
                q.put(finish)
        else:
            flag_start = 0
            node_map.map[temp_node.x, temp_node.y].val = d
            # print_node_mat(node_map, 2, 2)
            add_next(node_map, temp_node.x, temp_node.y)
            add_connected(node_map, q, temp_node)


def add_connected(node_map, queue, node):
     for i in [node.x - 1, node.x, node.x + 1]:
         for j in [node.y - 1, node.y, node.y + 1]:
             if i >= 0 and j >=0 and i < node_map.size_x and j < node_map.size_y:  # in the map
                if node_map.map[i, j].val == 0:  # first time in queue
                    node_map.map[i, j].val = -1
                    queue.put(node_map.map[i, j])


def distance_to_goal(loc_x, loc_y, gx, gy):
    return (loc_x - gx)**2 + (loc_y - gy)**2


def add_next(node_map, loc_x, loc_y):
    gx = node_map.gx
    gy = node_map.gy
    val = node_map.map[loc_x, loc_y].val
    min_val = val
    min_dis = node_map.size_x**2 + node_map.size_y**2
    for i in [loc_x - 1, loc_x, loc_x + 1]:
        for j in [loc_y - 1, loc_y, loc_y + 1]:
            if i >= 0 and j >= 0 and i < node_map.size_x and j < node_map.size_y:  # in the map
                temp_val = node_map.map[i, j].val
                if temp_val >= 2 and not(i == loc_x and j == loc_y):  # only free space that all ready have a number and not itself
                    if min_val >= temp_val:  # smallest val
                        if min_dis >= distance_to_goal(i, j, gx, gy):  # closest to goal
                            min_dis = distance_to_goal(i, j, gx, gy)
                            node_map.map[loc_x, loc_y].next_x = i
                            node_map.map[loc_x, loc_y].next_y = j
    next_x = node_map.map[loc_x, loc_y].next_x
    next_y = node_map.map[loc_x, loc_y].next_y
    if next_x != -1:
        node_map.map[next_x, next_y].prev.put(Cord(val, loc_x, loc_y))
        # print("(" + str(loc_x) + "," + str(loc_y) + ") v = " + str(val) + " to: (" +
        #       str(next_x) + "," + str(next_y) + ") v = " + str(node_map.map[next_x, next_y].val))


def root(node_map, robot_x, robot_y):
    temp_x, temp_y = world_to_map(node_map, robot_x, robot_y)
    val = node_map.map[temp_x, temp_y].val
    x = []
    y = []
    x_world = []
    y_world = []
    x.append(temp_x)
    y.append(temp_y)
    x_world.append(robot_x)
    y_world.append(robot_y)
    while val != 2:
        next_x = node_map.map[x[-1], y[-1]].next_x
        next_y = node_map.map[x[-1], y[-1]].next_y
        x.append(next_x)
        y.append(next_y)
        xx, yy = map_to_world(node_map, next_x, next_y)
        x_world.append(xx)
        y_world.append(yy)
        val = node_map.map[next_x, next_y].val

    # print_node_map(node_map, temp_x, temp_y)
    # plt.plot(x, y, "or")
    # plt.show()
    return x_world, x_world




if __name__ == '__main__':
    min_x = 0.0     # [cm]
    max_x = 50.0    # [cm]
    min_y = -25.0   # [cm]
    max_y = 25.0    # [cm]
    world_gx = 43.0  # [cm]
    world_gy = 15.0  # [cm]
    world_sx = 0.0
    world_sy = 0.0
    reso = 5        # [cm]
    node_map = NodeMap(min_x, max_x, min_y, max_y, world_gx, world_gy, reso)
    construct_node_map(node_map)

    # x, y = world_to_map(node_map, world_gx, world_gy)
    # xx, yy = map_to_world(node_map, x, y)

    # add obstical
    node_map.map[5, 5].val = 1.0
    node_map.map[5, 6].val = 1.0
    node_map.map[5, 4].val = 1.0
    node_map.map[6, 4].val = 1.0
    # wave_front
    wave_front(node_map)
    print_node_mat(node_map, 2, 2)
    # print_node_map(node_map, 2, 2)
    x, y = root(node_map, world_sx, world_sy)
    print_node_map(node_map, 2, 2)
