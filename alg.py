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
        self.q_obs = Queue.PriorityQueue()


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
    # print_node_mat(node_map)


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


def print_node_mat(node_map):
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
    flag_start = 0
    # iteration
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
            # print_node_mat(node_map)
            find_next(node_map, temp_node.x, temp_node.y)
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


def find_next(node_map, loc_x, loc_y):
    gx = node_map.gx
    gy = node_map.gy
    val = node_map.map[loc_x, loc_y].val
    if val == 2:  # the goal
        return 1
    min_val = val
    min_dis = node_map.size_x**2 + node_map.size_y**2
    for i in [loc_x - 1, loc_x, loc_x + 1]:
        for j in [loc_y - 1, loc_y, loc_y + 1]:
            if i >= 0 and j >= 0 and i < node_map.size_x and j < node_map.size_y:  # in the map
                temp_val = node_map.map[i, j].val
                if temp_val >= 2 and not(i == loc_x and j == loc_y):  # only free space that all ready have a number and not itself
                    if min_val >= temp_val:  # smallest val
                        if not(find_circle(node_map, loc_x, loc_y, i, j)):  # Next comes to current "Ping pong"
                            if min_dis >= distance_to_goal(i, j, gx, gy) or temp_val < min_val:  # closest to goal at lest one if exist
                                min_dis = distance_to_goal(i, j, gx, gy)
                                min_val = temp_val
                                node_map.map[loc_x, loc_y].next_x = i
                                node_map.map[loc_x, loc_y].next_y = j
    next_x = node_map.map[loc_x, loc_y].next_x
    next_y = node_map.map[loc_x, loc_y].next_y
    if next_x != -1:
        node_map.map[next_x, next_y].prev.put(Cord(distance_to_goal(loc_x, loc_y, gx, gy), loc_x, loc_y))
        # print("(" + str(loc_x) + "," + str(loc_y) + ") v = " + str(val) + " to: (" +
        #       str(next_x) + "," + str(next_y) + ") v = " + str(node_map.map[next_x, next_y].val))
        return 1  # There is a continuation
    else:
        node_map.map[loc_x, loc_y].val = -5  # sink node
        return 0  # sink node


def find_circle(node_map, cur_x, cor_y, next_x, next_y):
    next_next_x = node_map.map[next_x, next_y].next_x
    next_next_y = node_map.map[next_x, next_y].next_y
    if next_next_x != -1:
        hereafter_x = node_map.map[next_next_x, next_next_y].next_x
        hereafter_y = node_map.map[next_next_x, next_next_y].next_y
    else:
        hereafter_x = -1
        hereafter_y = -1

    if next_next_x == cur_x and next_next_y == cor_y:
        return 1  # There is a circle
    elif hereafter_x != -1:  # There is a continuation to continue
        if hereafter_x == cur_x and hereafter_y == cor_y:
            return 1  # There is a circle
    return 0  # No circle


def existing_route(node_map, robot_x, robot_y):
    val = node_map.map[robot_x, robot_y].val
    if val == 0:
        print('There is no route')
        return 0
    elif val == -5:  # inside sink
        print('need to re-calculates map')
        restart_map(node_map)
        wave_front(node_map)
        print_node_mat(node_map)  # TODO delet
        val = node_map.map[robot_x, robot_y].val
        if val == 0:
            print('There is no route')
            return 0
    return 1


def find_route(node_map, robot_x, robot_y):
    temp_x, temp_y = world_to_map(node_map, robot_x, robot_y)
    val = node_map.map[temp_x, temp_y].val
    x = []
    y = []
    x_world = []
    y_world = []
    if existing_route(node_map, temp_x, temp_y): # TODO Protect from robot placement on obstacle
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

        print_node_map(node_map, temp_x, temp_y)
        plt.plot(x, y, "or")
        plt.show()
    return x_world, x_world


def fictitious_magnification(node_map, obs_x, obs_y, gx, gy):
    val = node_map.map[obs_x, obs_y].val
    if not(val == -10.0 or val == 1):  # the node is not in the queue and not a wall
        for i in [obs_x - 1, obs_x, obs_x + 1]:
            for j in [obs_y - 1, obs_y, obs_y + 1]:
                if i >= 0 and j >= 0 and i < node_map.size_x and j < node_map.size_y:  # in the map
                    val = node_map.map[i, j].val
                    if not(val == -10.0 or val == 1):  # the node is not in the queue and not a wall
                        node_map.map[i, j].val = -10.0  # the node is put inside the queue
                        node_map.q_obs.put(Cord(distance_to_goal(i, j, gx, gy), i, j))


def check_new_obstacle(node_map, world_obs_x, world_obs_y):
    obs_x, obs_y = world_to_map(node_map, world_obs_x, world_obs_y)
    gx = node_map.gx
    gy = node_map.gy
    val = int(round(node_map.map[obs_x, obs_y].val))
    if obs_x < 0 or obs_y < 0 or obs_x >= node_map.size_x or obs_y >= node_map.size_y:  # out of map
        return -1  # out of boundary
    elif val == 1:  # old obstacle
        if node_map.q_obs.empty():  # There were no new walls
            return 0  # old obstacle
        else:  # There were new walls
            add_new_obstacle(node_map)
            return 2  # need to updating route
    else:
        fictitious_magnification(node_map, obs_x, obs_y, gx, gy)
        if node_map.q_obs.qsize() < 10:  # TODO optimize
            return 1  # new obstacle
        else:
            add_new_obstacle(node_map)
            return 2  # need to updating route


def add_new_obstacle(node_map):
    q_new_direction = Queue.PriorityQueue()
    while not node_map.q_obs.empty():  # as long as there is a new obstacle
        cord = node_map.q_obs.get()
        node_map.map[cord.x, cord.y].val = 1.0
        node_map.map[cord.x, cord.y].next_x = -1.0
        node_map.map[cord.x, cord.y].next_y = -1.0
        while not node_map.map[cord.x, cord.y].prev.empty():  # As long as there is a node that passed through it
            q_new_direction.put(node_map.map[cord.x, cord.y].prev.get())
    find_new_next(node_map, q_new_direction)


def find_new_next(node_map, q_new_direction):  # find new next fore the node connected to the new wall
    while not q_new_direction.empty():
        cord = q_new_direction.get()
        if node_map.map[cord.x, cord.y].val != 1:  # not a wall
            node_map.map[cord.x, cord.y].next_x = -1
            node_map.map[cord.x, cord.y].next_y = -1
            if not(find_next(node_map, cord.x, cord.y)):  # it is sink node
                while not node_map.map[cord.x, cord.y].prev.empty():  # As long as there's a node that passed through it
                    q_new_direction.put(node_map.map[cord.x, cord.y].prev.get())
    # print_node_mat(node_map)


def restart_map(node_map):
    # print_node_mat(node_map)
    size_x = node_map.size_x
    size_y = node_map.size_y
    for i in range(size_x):
        for j in range(size_y):
            if node_map.map[i, j].val != 1 and node_map.map[i, j].val != 2:
                node_map.map[i, j].val = 0

    # print_node_map(node_map, 2, 2)
    # print_node_mat(node_map)


def simulate(min_x, max_x, min_y, max_y, world_gx, world_gy, world_sx, world_sy, reso):
    # Creates the map
    node_map = NodeMap(min_x, max_x, min_y, max_y, world_gx, world_gy, reso)
    # Constructive the map
    construct_node_map(node_map)
    # Add obstacles
    sim_add_obstacles(node_map, 1)
    # Wave_front
    wave_front(node_map)
    # Print map
    print_node_mat(node_map)
    # Todo This part should be in a loop
    # Find_route
    x, y = find_route(node_map, world_sx, world_sy)
    if not(len(x)):
        print('There is no route - stop all')
    # Start to drive
        # TODO
    # Adding obstacles along the way
    sim_add_obstacles(node_map, 2)
    # Find_route
    x, y = find_route(node_map, 25, world_sy)
    if not(len(x)):
        print('There is no route - stop all')
    # Start to drive
    # TODO


def sim_add_obstacles(node_map, time):
    if time == 1:
        # add obstacles one way
        node_map.map[3, 8].val = 1.0
        node_map.map[4, 8].val = 1.0
        node_map.map[5, 8].val = 1.0
        node_map.map[6, 8].val = 1.0
        node_map.map[7, 8].val = 1.0
        node_map.map[8, 8].val = 1.0
    elif time == 2:
        # add obstacles another way
        # TODO need to write in loop
        world_obs_x = 35
        world_obs_y = 15
        a = check_new_obstacle(node_map, world_obs_x, world_obs_y)
        a = check_new_obstacle(node_map, world_obs_x, world_obs_y + 5)
        print_node_mat(node_map)
        a = check_new_obstacle(node_map, world_obs_x, world_obs_y - 5)
        # a = check_new_obstacle(node_map, 20, 0)
        # a = check_new_obstacle(node_map, world_obs_x, world_obs_y + 10)
        # a = check_new_obstacle(node_map, world_obs_x, world_obs_y + 15)
        # a = check_new_obstacle(node_map, world_obs_x, world_obs_y + 20)
        # a = check_new_obstacle(node_map, world_obs_x - 5, world_obs_y)
        # a = check_new_obstacle(node_map, world_obs_x - 5, world_obs_y - 5)
        # a = check_new_obstacle(node_map, world_obs_x - 5, world_obs_y - 15)
        # a = check_new_obstacle(node_map, world_obs_x - 5, world_obs_y - 20)
        # a = check_new_obstacle(node_map, world_obs_x - 10, world_obs_y - 20)
        # a = check_new_obstacle(node_map, world_obs_x - 15, world_obs_y - 20)
        # a = check_new_obstacle(node_map, world_obs_x - 20, world_obs_y - 20)
        print_node_mat(node_map)


if __name__ == '__main__':
    min_x = 0.0     # [cm]
    max_x = 50.0    # [cm]
    min_y = -25.0   # [cm]
    max_y = 25.0    # [cm]
    world_gx = 43.0  # [cm]
    world_gy = 15.0  # [cm]
    world_sx = 0.0
    world_sy = 15.0
    reso = 5        # [cm]
    simulate(min_x, max_x, min_y, max_y, world_gx, world_gy, world_sx, world_sy, reso)

    # x, y = world_to_map(node_map, world_gx, world_gy)
    # xx, yy = map_to_world(node_map, x, y)

