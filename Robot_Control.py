from Rclient import RClient
import numpy as np
import cubic_spline_planner
from time import time
from time import sleep
import matplotlib.pyplot as plt

# Variables
f = 1
Closeness0 = 10  # cm
Closeness45 = 7  # cm
commands_in_a_row = 10
Kp = 1.0  # speed propotional gain
ds = 0.1
dt = 0.01
k = 0.5  # control gain


class Robot(object):
    def __init__(self):
        super(Robot, self).__init__()
        self.state = RClient("192.168.1.151", 2777)
        self.state.connect()
        self.x = 0.0
        self.y = 0.0
        self.Dx = 0.0
        self.Dy = 0.0
        self.Obs0 = 0.0
        self.Obs45L = 0.0
        self.Obs45R = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.dt = 0.0
        self.time_stamp = time()
        self.X = []
        self.Y = []
        self.YAW = []
        self.V = []
        self.T = [0]
        self.update_params()

    def update_params(self):
        x, y, self.Dx, self.Dy, self.Obs0, self.Obs45L, self.Obs45R = self.state.sense()
        self.yaw = (np.arcsin(self.Dy)/abs(np.arcsin(self.Dy))) * np.arccos(self.Dx)
        self.dt = time() - self.time_stamp
        self.time_stamp = time()
        self.v = np.sqrt((x - self.x)**2 + (y - self.y)**2) / self.dt
        self.x = x
        self.y = y
        self.X.append(self.x)
        self.Y.append(self.y)
        self.YAW.append(self.yaw)
        self.V.append(self.v)
        self.T.append(self.T[-1] + self.dt)

    def drive(self, left, right):
        self.state.drive(left, right)

    def plot_actual_motion(self):
        plt.plot(self.X, self.Y, ".r", label="course")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)
        plt.show()

        plt.plot(self.T, self.V, "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()

        plt.plot(self.T, self.YAW, "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Yaw[deg]")
        plt.grid(True)
        plt.show()


class MotionProfile(object):
    def __init__(self):
        super(MotionProfile, self).__init__()
        self.X = []
        self.Y = []
        self.yaw = []
        self.v = []
        self.left = []
        self.right = []

    def update_profile(self, X, Y, yaw, v, left, right):
        self.X = X
        self.Y = Y
        self.yaw = yaw
        self.v = v
        self.left = left
        self.right = right


robot = Robot()
profile = MotionProfile()


def stanley_control(last_target_idx):
    current_target_idx, error_front_axle = calc_target_index()

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = profile.yaw[current_target_idx] - robot.yaw
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, robot.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


def calc_target_index():
    # Calc front axle position
    fx = robot.x
    fy = robot.y

    # Search nearest point index
    dx = [fx - icx for icx in profile.X]
    dy = [fy - icy for icy in profile.Y]
    d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
    closest_error = min(d)
    target_idx = d.index(closest_error)

    # Project RMS error onto front axle vector
    front_axle_vec = [robot.Dx, robot.Dy]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle


def pid_control(target, current):
    return Kp * (target - current)


def create_motion_profile(x, y):
    return cubic_spline_planner.calc_spline_course(x, y, ds)


def plot_motion_profile(x, y, X, Y, yaw, s):
    plt.subplots(1)
    plt.plot(x, y, "xb", label="input")
    plt.plot(X, Y, "-r", label="spline")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()

    plt.subplots(1)
    plt.plot(s, [np.rad2deg(iyaw) for iyaw in yaw], "-r", label="yaw")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("yaw angle[deg]")

    plt.show()


def convert_angle_and_velocity_to_wheels_commends(delta_yaw, v):
    DC = 500
    normalize_yaw = (delta_yaw / np.pi) * 10
    left = DC - normalize_yaw
    right = DC + normalize_yaw
    return left, right


def preform_motion_profile():
    check_position()
    target_idx, _ = calc_target_index()

    last_idx = len(profile.X) - 1
    while last_idx > target_idx:
        a = pid_control(profile.v[target_idx], robot.v)
        delta_yaw, target_idx = stanley_control(target_idx)
        left, right = convert_angle_and_velocity_to_wheels_commends(delta_yaw, a*robot.dt)
        robot.drive(int(left), int(right))
        sleep(0.5)
        robot.update_params()
        check_position()


def check_position():
    # handling new obstacle.
    at0, at45L, at45R = facing_new_obstacle()
    if at0 or at45L or at45L:
        pass_obstacle()

    # handling the situation the we are too close to an obstacle.
    if are_we_too_close():
        drive_back_and_recalculate_route()


def facing_new_obstacle():
    at0 = at45L = at45R = 0

    if robot.Obs0 != 0:
        at0 = check_obstacle(robot.x, robot.y, np.arctan2(robot.y, robot.x), robot.Obs0, 0)
    if robot.Obs45L != 0:
        at45L = check_obstacle(robot.x, robot.y, np.arctan2(robot.y, robot.x), robot.Obs45L, 45)
    if robot.Obs45R != 0:
        at45R = check_obstacle(robot.x, robot.y, np.arctan2(robot.y, robot.x), robot.Obs45R, -45)

    return at0, at45L, at45R


def create_target_velocity_profile(c, s):
    c = np.abs(c) / np.max(np.abs(c))
    a = np.array([1 - ai for ai in c])

    plt.plot(s, a, ".r", label="a")
    plt.legend()
    plt.xlabel("motion length[m]")
    plt.ylabel("a[m/s^2]")
    plt.grid(True)
    plt.show()
    return a


def check_obstacle(robot_x, robot_y, robot_angle, obs_dis, obs_angle):
    obs_x = robot_x + obs_dis*np.cos(robot_angle + obs_angle)
    obs_y = robot_y + obs_dis*np.sin(robot_angle + obs_angle)
    return 0


def are_we_too_close():
    return robot.Obs0 <= Closeness0 or robot.Obs45L <= Closeness45 or robot.Obs45R <= Closeness45


def drive_back_and_recalculate_route():
    print()


def pass_obstacle():
    print()


def map_robot_wheels_commends_to_yaw():
    yaw_map = np.array([0, 0, 0, 0])
    for i in range(300, 1000, 10):
        for j in range(300, 1000, 10):
            robot.drive(int(i), int(j))
            robot.update_params()
            sleep(1)
            yaw_map = np.vstack((yaw_map, np.array([i, j, robot.Dx, robot.Dy])))
    return yaw_map



def main():
    yaw_map = map_robot_wheels_commends_to_yaw()
    xi = robot.x
    yi = robot.y

    x = [xi] + [0.0, 100.0, 100.0, 50.0, 60.0]
    y = [yi] + [0.0, 0.0, -30.0, -20.0, 0.0]

    X, Y, yaw, ck, s = create_motion_profile(x, y)
    plot_motion_profile(x, y, X, Y, yaw, s)
    v = create_target_velocity_profile(ck, s)
    # left, right = convert_angle_and_velocity_to_wheels_commends(yaw, a)
    profile.update_profile(X, Y, yaw, v)

    preform_motion_profile()

if __name__ == '__main__':
    main()