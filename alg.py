import matplotlib.pyplot as plt
import numpy as np
import Queue
# from Robot_Main import world_gy, world_gx, obs_vec_x, obs_vec_y
import math

# Global
min_x = 0.0     # [cm]
max_x = 100.0  # [cm]
min_y = 0.0  # [cm]
max_y = 100   # [cm]
# min_x = 0.0     # [cm]
# max_x = 10.0  # [cm]
# min_y = 0.0  # [cm]
# max_y = 10   # [cm]
reso = 10        # [cm]

world_gx = 0  # [cm]
world_gy = 0  # [cm]
# obs_vec_x = [-65, -55, -45, -35, -25, -15, -5,   5,   10,  15,  25,  35,  45,  55,  65, -65, -55, -45, -35, -25, -15, -5,   5,   10,  15,  25,  35,  45,  55,  65]
# obs_vec_y = [-62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72]
obs_vec_x = []
obs_vec_y = []

# Class
class Node:

    def __init__(self, x, y, value):
        self.x = x
        self.y = y
        self.val = value
        self.next_x = -1
        self.next_y = -1
        self.prev = Queue.PriorityQueue()
        self.yaw = 0

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
        self.gx, self.gy = world_to_map(world_gx, world_gy)
        self.q_obs = Queue.PriorityQueue()
        self.q_base_obs = Queue.Queue()
        self.x_route = []
        self.y_route = []
        self.obs_size = 2  # [pixel]


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


# Auxiliary functions
def line_cross_over_obstacle_on_way_to_target_in_world(x_start, y_start):
    x1, y1 = world_to_map(x_start, y_start)
    x2, y2 = world_to_map(world_gx, world_gy)
    return line_cross_over_obstacle_in_map(x1, y1, x2, y2)


def line_cross_over_obstacle_in_map(x_start, y_start, x_end, y_end):
    # print_node_map(node_map.x_route[0], node_map.y_route[0])
    flag = False
    map_node = node_map.map

    if (x_end - x_start) != 0:
        a = float(y_end - y_start) / float(x_end - x_start)
        if abs(y_end - y_start) <= abs(x_end - x_start):
            for i in np.arange(0, (x_end - x_start), 0.01*np.sign((x_end - x_start))):
                xi = x_start + int(round(i))
                yi = y_start + int(round(a*i))
                value = map_node[xi, yi].val
                if map_node[xi, yi].val == 1:
                    flag = True
                #     plt.plot(xi,  yi, "or")
                # else:
                #     plt.plot(xi, yi, "ok")
        else:
            for i in np.arange(0, (y_end - y_start), 0.01*np.sign((y_end - y_start))):
                xi = x_start + int(round(i/a))
                yi = y_start + int(round(i))
                value = map_node[xi, yi].val
                if map_node[xi, yi].val == 1:
                    flag = True
                #     plt.plot(xi,  yi, "or")
                # else:
                #     plt.plot(xi, yi, "ok")
    else:
        for i in np.arange(0, (y_end - y_start), 0.01*np.sign((y_end - y_start))):
            yi = y_start + int(round(i))
            value = map_node[x_start, yi].val
            if map_node[x_start, yi].val == 1:
                flag = True
                # plt.plot(x_start,  yi, "or")
            # else:
                # plt.plot(x_start, yi, "ok")
    # plt.grid(True)
    # plt.show()
    return flag


def calc_map_size(max_val, min_val, resolution):
    return int(round((max_val - min_val) / resolution)) + 2


def world_to_map(point_x, point_y):
    map_point_x = int(round((point_x - min_x) / reso)) + 1
    map_point_y = int(round((point_y - min_y) / reso)) + 1
    return map_point_x, map_point_y


def map_to_world(point_x, point_y):
    world_point_x = (point_x - 1) * reso + min_x
    world_point_y = (point_y - 1) * reso + min_y
    return world_point_x, world_point_y


def distance_to_goal(loc_x, loc_y):
    return (loc_x - world_gx)**2 + (loc_y - world_gy)**2


def distance_to_goal_in_map(loc_x, loc_y, gx, gy):
    return (loc_x - gx)**2 + (loc_y - gy)**2


# print functions
def print_node_map(robot_x, robot_y):
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


def print_node_world(robot_x, robot_y):
    map_node = node_map.map
    size_x = node_map.size_x
    size_y = node_map.size_y
    for i in range(size_x):
        for j in range(size_y):
            if map_node[i, j].val == 1:
                plt.plot(map_to_world(i, j), ".k")
            elif map_node[i, j].val == 2:
                plt.plot(map_to_world(i, j), "xb")
            elif i == robot_x and j == robot_y:
                plt.plot(map_to_world(i, j), "or")
    plt.grid(True)
    return plt


def print_node_mat():
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


def print_arrow_mat():
    print_node_map(node_map.x_route[0], node_map.y_route[0])
    map_node = node_map.map
    size_x = node_map.size_x
    size_y = node_map.size_y
    for i in range(size_x):
        for j in range(size_y):
            dx = 0.5 * np.cos(map_node[i, j].yaw)
            dy = 0.5 * np.sin(map_node[i, j].yaw)
            plt.arrow(i, j, dx, dy, width=0.1)
    plt.show()


def print_arrow_yaw_mat(yaw_mat):
    print_node_map(node_map.x_route[0], node_map.y_route[0])
    size_x = node_map.size_x
    size_y = node_map.size_y
    for i in range(size_x):
        for j in range(size_y):
            dx = 0.5 * np.cos(yaw_mat[i, j])
            dy = 0.5 * np.sin(yaw_mat[i, j])
            plt.arrow(i, j, dx, dy, width=0.1)
    plt.show()


# Create the map - 1 MODEL
def create_map():
    # Constructive the map
    construct_node_map()
    # initialize obstacle queue
    initialize_obstacle_queue(obs_vec_x, obs_vec_y)
    # Add obstacles
    add_new_obstacle()
    # Wave_front
    wave_front()
    return


# Construct node map - 1.1 MODEL
def construct_node_map():
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
    # print_node_map(2, 2)
    # print_node_mat()


# initialize obstacle queue
def initialize_obstacle_queue(obs_vec_x, obs_vec_y):
    for i in range(len(obs_vec_x)):
        check_new_obstacle(obs_vec_x[i], obs_vec_y[i])


# Add new obstacle - 1.2 MODEL
def add_new_obstacle():
    q_new_direction = Queue.PriorityQueue()
    while not node_map.q_obs.empty():  # as long as there is a new obstacle
        cord = node_map.q_obs.get()
        node_map.map[cord.x, cord.y].val = 1.0
        node_map.map[cord.x, cord.y].next_x = -1.0
        node_map.map[cord.x, cord.y].next_y = -1.0
        while not node_map.map[cord.x, cord.y].prev.empty():  # As long as there is a node that passed through it
            q_new_direction.put(node_map.map[cord.x, cord.y].prev.get())
    find_new_next(q_new_direction)


def find_new_next(q_new_direction):  # find new next fore the node connected to the new wall
    while not q_new_direction.empty():
        cord = q_new_direction.get()
        if node_map.map[cord.x, cord.y].val != 1:  # not a wall
            node_map.map[cord.x, cord.y].next_x = -1
            node_map.map[cord.x, cord.y].next_y = -1
            if not(find_next(cord.x, cord.y)):  # it is sink node
                while not node_map.map[cord.x, cord.y].prev.empty():  # As long as there's a node that passed through it
                    q_new_direction.put(node_map.map[cord.x, cord.y].prev.get())
    # print_node_mat()


# Wave front - 1.3 MODEL
def wave_front():
    # initialize
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
            # print_node_mat()
            find_next(temp_node.x, temp_node.y)
            add_connected(q, temp_node)


def find_next(loc_x, loc_y):
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
                        if not(find_circle(loc_x, loc_y, i, j)):  # Next comes to current "Ping pong"
                            temp_dis = distance_to_goal_in_map(i, j, gx, gy)
                            if min_dis >= temp_dis or temp_val < min_val:  # closest to goal at lest one if exist
                                min_dis = temp_dis
                                min_val = temp_val
                                node_map.map[loc_x, loc_y].next_x = i
                                node_map.map[loc_x, loc_y].next_y = j
    next_x = node_map.map[loc_x, loc_y].next_x
    next_y = node_map.map[loc_x, loc_y].next_y
    if next_x != -1:
        node_map.map[next_x, next_y].prev.put(Cord(distance_to_goal_in_map(loc_x, loc_y, gx, gy), loc_x, loc_y))
        # print("(" + str(loc_x) + "," + str(loc_y) + ") v = " + str(val) + " to: (" +
        #       str(next_x) + "," + str(next_y) + ") v = " + str(node_map.map[next_x, next_y].val))
        return 1  # There is a continuation
    else:
        node_map.map[loc_x, loc_y].val = -5  # sink node
        return 0  # sink node


def find_circle(cur_x, cor_y, next_x, next_y):
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


def add_connected(queue, node):
     for i in [node.x - 1, node.x, node.x + 1]:
         for j in [node.y - 1, node.y, node.y + 1]:
             if i >= 0 and j >=0 and i < node_map.size_x and j < node_map.size_y:  # in the map
                if node_map.map[i, j].val == 0:  # first time in queue
                    node_map.map[i, j].val = -1
                    queue.put(node_map.map[i, j])


# Calculate_route - 2 MODEL
def calculate_route(world_robot_x, world_robot_y):
    robot_x, robot_y = world_to_map(world_robot_x, world_robot_y)
    yaw_mat = np.array([])
    x_world = []
    y_world = []
    add_new_obstacle()
    print_node_map(robot_x, robot_y)
    plt.plot(robot_x, robot_y, 'or')
    plt.show()
    flag, robot_x, robot_y = existing_route(robot_x, robot_y)
    if not(flag):
        print('There is no route')
        return x_world, y_world, yaw_mat
    x_world, y_world = find_route(robot_x, robot_y, world_robot_x, world_robot_y)
    yaw_mat = calc_yaw_all_map()
    return x_world, y_world, yaw_mat


def existing_route(robot_x, robot_y):
    val = node_map.map[robot_x, robot_y].val
    if node_map.obs_size > 0:
        if val == 0 or val == 1 or val == -5:
            if val == 1:
                flag, robot_x, robot_y = find_out_from_obs(robot_x, robot_y)
                if flag:
                    return 1, robot_x, robot_y
                else:
                    node_map.obs_size -= 1
                    print('obs_size: ' + str(node_map.obs_size))
            elif val == 0:
                node_map.obs_size -= 1
                print('obs_size: ' + str(node_map.obs_size))
            print('need to re-calculates map')
            re_initialize_map()
            print_node_mat()
            flag, robot_x, robot_y = existing_route(robot_x, robot_y)
            if not (flag):
                return 0, robot_x, robot_y
    else:
        return 0, robot_x, robot_y
    return 1, robot_x, robot_y


def find_out_from_obs(robot_x, robot_y):
    obs_size = node_map.obs_size
    new_robot_x = robot_x
    new_robot_y = robot_y
    min_dis = (obs_size + 3)  # TODO **2
    factor = 1
    flag_found = False
    while not(flag_found) and (factor <= obs_size + 1):
        for i in range(robot_x - factor, robot_x + factor + 1):
            for j in range(robot_y - factor, robot_y + factor + 1):
                if i >= 0 and j >= 0 and i < node_map.size_x and j < node_map.size_y:  # in the map
                    boundary_min_x = robot_x - factor
                    boundary_max_x = robot_x + factor
                    boundary_min_y = robot_y - factor
                    boundary_max_y = robot_y + factor
                    # on the frame
                    if i == boundary_min_x or j == boundary_min_y or i == boundary_max_x or j == boundary_max_y:
                        val = node_map.map[i, j].val
                        if val != 1 and val != 0:
                            temp_dis = distance_to_goal_in_map(i, j, robot_x, robot_y)
                            if min_dis >= temp_dis:  # closest to robot at lest one if exist
                                min_dis = temp_dis
                                new_robot_x = node_map.map[i, j].x
                                new_robot_y = node_map.map[i, j].y
                                flag_found = True
        factor += 1
    if flag_found:
        return 1, new_robot_x, new_robot_y
    return 0, new_robot_x, new_robot_y


def re_initialize_map():
    restart_map()
    re_initialize_obstacle_queue()
    add_new_obstacle()
    wave_front()


def re_initialize_obstacle_queue():
    q = Queue.PriorityQueue()
    gx = node_map.gx
    gy = node_map.gy
    while not node_map.q_base_obs.empty():
        cord = node_map.q_base_obs.get()
        fictitious_magnification(cord.x, cord.y, gx, gy)
        q.put(Cord(distance_to_goal_in_map(cord.x, cord.y, gx, gy), cord.x, cord.y))
    node_map.q_base_obs = q


# find route - 2.1 MODEL
def find_route(temp_x, temp_y, world_robot_x, world_robot_y):
    x_world = []
    y_world = []
    if not(temp_x >= 0 and temp_y >= 0 and temp_x < node_map.size_x and temp_y < node_map.size_y):  # in the map
        print('Out of Map')
        return x_world, y_world
    val = node_map.map[temp_x, temp_y].val
    x_map = []
    y_map = []

    x_map.append(temp_x)
    y_map.append(temp_y)
    x_world.append(world_robot_x)
    y_world.append(world_robot_y)
    while val != 2:
        next_x = node_map.map[x_map[-1], y_map[-1]].next_x
        next_y = node_map.map[x_map[-1], y_map[-1]].next_y
        x_map.append(next_x)
        y_map.append(next_y)
        val = node_map.map[next_x, next_y].val
    node_map.x_route = x_map
    node_map.y_route = y_map

    calc_straight_line_route()
    print_node_map(temp_x, temp_y)
    plt.plot(node_map.x_route, node_map.y_route, 'or')
    plt.show()

    for xx, yy in zip(node_map.x_route, node_map.y_route):
        xx_w, yy_w = map_to_world(xx, yy)
        x_world.append(xx_w)
        y_world.append(yy_w)
    return x_world, y_world


def calc_straight_line_route():
    new_route_x = [node_map.x_route[0]]
    new_route_y = [node_map.y_route[0]]
    i = 0
    while i < len(node_map.x_route) - 1:
        x = node_map.x_route[i]
        y = node_map.y_route[i]
        z = i

        while z < len(node_map.x_route) - 1:
            if line_cross_over_obstacle_in_map(x, y, node_map.x_route[z + 1], node_map.y_route[z + 1]):
                break
            else:
                z = z + 1
        i = z
        new_route_x.append(node_map.x_route[i])
        new_route_y.append(node_map.y_route[i])

    node_map.x_route = new_route_x
    node_map.y_route = new_route_y


def restart_map():
    # print_node_mat()
    size_x = node_map.size_x
    size_y = node_map.size_y
    for i in range(size_x):
        for j in range(size_y):
            # if node_map.map[i, j].val != 1 and node_map.map[i, j].val != 2:
            if node_map.map[i, j].val != 2:
                if not(i == 0 or j == 0 or i == (size_x - 1) or j == (size_y - 1)):
                    node_map.map[i, j].val = 0

    # print_node_map(2, 2)
    # print_node_mat()


# Calc yaw - 2.4 MODEL
def calc_yaw_all_map():
    size_x = node_map.size_x
    size_y = node_map.size_y
    yaw_mat = np.zeros((size_x, size_y))
    for i in range(size_x):
        for j in range(size_y):
            if node_map.map[i, j].val == 1:
                next_idx = 0
                yaw = calc_yaw(i, j, next_idx)
            else:
                next_idx = 1
                yaw = calc_yaw(i, j, next_idx)
            node_map.map[i, j].yaw = yaw
            yaw_mat[i, j] = yaw
    return yaw_mat


def calc_yaw(loc_x, loc_y, next_idx):
    x, y = find_closest_node_in_route(loc_x, loc_y, next_idx)
    yaw = (np.arctan2(y - loc_y, x - loc_x))
    return yaw


def find_closest_node_in_route(loc_x, loc_y, next_idx):
    x_map = node_map.x_route
    y_map = node_map.y_route
    # Search nearest point index
    dx = [loc_x - icx for icx in x_map]
    dy = [loc_y - icy for icy in y_map]
    d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
    closest_in_route = min(d)
    closest_idx_in_route = d.index(closest_in_route)
    direction_idx = closest_idx_in_route + next_idx
    if direction_idx >= (len(x_map) - 1):
        direction_idx = (len(x_map) - 1)
    x = x_map[direction_idx]
    y = y_map[direction_idx]
    return x, y


# Check_new_obstacle - 3 MODEL
def check_new_obstacle(world_obs_x, world_obs_y):
    obs_x, obs_y = world_to_map(world_obs_x, world_obs_y)
    gx = node_map.gx
    gy = node_map.gy
    if obs_x < 0 or obs_y < 0 or obs_x >= node_map.size_x or obs_y >= node_map.size_y:  # out of map
        return -1  # out of boundary
    val = int(round(node_map.map[obs_x, obs_y].val))
    if val == 1:  # old obstacle
        return 0  # old obstacle
    else:
        node_map.q_base_obs.put(Cord(distance_to_goal_in_map(obs_x, obs_y, gx, gy), obs_x, obs_y))
        fictitious_magnification(obs_x, obs_y, gx, gy)
        return 1  # new obstacle


def fictitious_magnification(obs_x, obs_y, gx, gy):
    val = node_map.map[obs_x, obs_y].val
    obs_size = node_map.obs_size
    if not(val == -10.0 or val == 1):  # the node is not in the queue and not a wall
        for i in range(obs_x - obs_size, obs_x + obs_size + 1):
            for j in range(obs_y - obs_size, obs_y + obs_size + 1):
                if i >= 0 and j >= 0 and i < node_map.size_x and j < node_map.size_y:  # in the map
                    val = node_map.map[i, j].val
                    if not(val == -10.0 or val == 1):  # the node is not in the queue and not a wall
                        node_map.map[i, j].val = -10.0  # the node is put inside the queue
                        node_map.q_obs.put(Cord(distance_to_goal_in_map(i, j, gx, gy), i, j))


def front_or_back_obstacle(world_robot_x, world_robot_y, world_obs_x, world_obs_y):
    robot_x, robot_y = world_to_map(world_robot_x, world_robot_y)
    obs_x, obs_y = world_to_map(world_obs_x, world_obs_y)
    vr = node_map.map[robot_x, robot_y].val
    vo = node_map.map[obs_x, obs_y].val
    print('robot val: ' + str(node_map.map[robot_x, robot_y].val) + ', obj val: ' + str(node_map.map[obs_x, obs_y].val))
    return node_map.map[robot_x, robot_y].val <= node_map.map[obs_x, obs_y].val


# Global map
node_map = NodeMap(min_x, max_x, min_y, max_y, world_gx, world_gy, reso)


# Simulate
def simulate():
    # parameters
    world_sx = 90
    world_sy = 90
    node_map1 = node_map

    # run
    create_map()
    print_node_mat()
    x_world, y_world, yaw_mat = calculate_route(world_sx, world_sy)
    print_arrow_mat()
    print_arrow_yaw_mat(yaw_mat)

    world_robot_x = 20
    # world_robot_y = 20
    # world_obs_x = 30
    # world_obs_y = 30
    # a = front_or_back_obstacle(world_robot_x, world_robot_y, world_obs_x, world_obs_y)
    # print_node_mat()
    # check kine corve
    # x1, y1 = world_to_map(world_sx, world_sy)
    # x2, y2 = world_to_map(-50, 0)
    # line_cross_over_obstacle_in_map(x1, y1, x2, y2)
    # line_cross_over_obstacle_on_way_to_target_in_world(world_sx, world_sy)
    # obs = check_new_obstacle
    # print_arrow_mat()
    # print_node_mat()



if __name__ == '__main__':
    simulate()

