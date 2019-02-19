from Rclient import RClient
import numpy as np
import cubic_spline_planner
from time import time
from time import sleep
import matplotlib.pyplot as plt

# Variables
f = 1
Closeness0 = 45.0  # [cm]
Closeness45 = 45.0  # [cm]
Kp = 1.0  # speed proportional gain
ds = 10  # [cm] Quantization of the motion
k = 0.5  # control gain
initial_yaw_delta = 10.0  # [deg]
final_position_delta = 10.0  # [cm]
sweeping_angle = 375.0  # [cm]
coursing_velocity = 400.0  # [wheel power]
stop_velocity = -300  # [wheel power]
sleep_time = 0.23  # [sec]
velocity_gain = 700  # [wheel power]
velocity_offset = 330  # [wheel power]
next_idx = 1
pass_obstacle_timeout = 10  # [sec]


class Robot(object):
    def __init__(self):
        super(Robot, self).__init__()
        self.state = RClient("192.168.1.153", 2777)
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
        self.csv_file = np.array([self.x, self.y, self.Dx, self.Dy, self.yaw, self.v, self.dt])
        sleep(1)
        self.update_params()
        sleep(1)

    def update_params(self):
        x, y, self.Dx, self.Dy, self.Obs45L, self.Obs0, self.Obs45R = self.state.sense()
        # self.yaw = (np.arcsin(self.Dy)/abs(np.arcsin(self.Dy))) * np.arccos(self.Dx)
        self.yaw = np.rad2deg(np.arctan2(self.Dy, self.Dx))
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
        self.csv_file = np.vstack((self.csv_file, np.array([self.x, self.y, self.Dx, self.Dy, self.yaw, self.v, self.dt])))

    def drive(self, delta_yaw, v):
        left, right = convert_angle_and_velocity_to_wheels_commends(delta_yaw, v)
        self.state.drive(int(left), int(right))
        sleep(sleep_time)
        robot.update_params()

    def plot_actual_motion(self):
        plt.plot(self.X, self.Y, label="course")
        plt.plot(profile.X, profile.Y, "r", label="spline")
        plt.legend()
        plt.title('actual X-Y coordinates')
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)
        plt.show()

        plt.plot(self.T, self.V)
        plt.title('actual velocity')
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()

        plt.plot(self.T, self.YAW)
        plt.title('actual yaw')
        plt.xlabel("Time[s]")
        plt.ylabel("Yaw[deg]")
        plt.grid(True)
        plt.show()

        np.savetxt("actual motion/robot -" + str(time()) + ".csv", self.csv_file, delimiter=",")


class MotionProfile(object):
    def __init__(self):
        super(MotionProfile, self).__init__()
        self.X = []
        self.Y = []
        self.yaw = []
        self.v = []
        self.s = []
        self.last_idx = 0

    def update_profile(self, X, Y, yaw, v, s):
        self.X = X
        self.Y = Y
        self.yaw = yaw
        self.v = v
        self.s = s
        self.last_idx = len(profile.X) - 1

    def plot_motion_profile(self, x, y):
        plt.subplots(1)
        plt.plot(x, y, "xb", label="input")
        plt.plot(self.X, self.Y, "r", label="spline")
        plt.title('plan motion - X-Y coordinates')
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots(1)
        plt.plot(self.s, self.yaw, "r", label="yaw")
        plt.title('plan motion - yaw')
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("yaw angle[deg]")

        plt.show()


robot = Robot()
profile = MotionProfile()


def create_motion_profile(x, y):
    return cubic_spline_planner.calc_spline_course(x, y, ds)


def create_target_velocity_profile(c, s): # TODO improve
    c = np.abs(c) / np.max(np.abs(c))
    v = np.array([(1 - ai) * velocity_gain + velocity_offset for ai in c])

    plt.plot(s, v, label="a")
    plt.legend()
    plt.title('target velocity profile')
    plt.xlabel("motion length[m]")
    plt.ylabel("a[m/s^2]")
    plt.grid(True)
    plt.show()
    return v


def convert_angle_and_velocity_to_wheels_commends(delta_yaw, v):
    left = v + delta_yaw
    right = v - delta_yaw

    if np.abs(left) > 1000:  # TODO add condition on 0
        left = np.sign(left)*1000
    if np.abs(right) > 1000:
        right = np.sign(right)*1000
    return left, right


def stanley_control(last_target_idx):
    current_target_idx = calc_target_index()

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = profile.yaw[current_target_idx] - robot.yaw

    # theta_d corrects the cross track error
    # theta_d = np.arctan2(k * error_front_axle, robot.v)
    x_hat = profile.X[current_target_idx]+k*np.cos(profile.yaw[current_target_idx]) - robot.x
    y_hat = profile.Y[current_target_idx]+k*np.sin(profile.yaw[current_target_idx]) - robot.y
    theta_d = np.arctan2(y_hat, x_hat) * 10

    print('theta_e: ' + str(theta_e))
    print('theta_d: ' + str(theta_d))
    # Steering control
    delta = theta_e - theta_d

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
    target_idx = d.index(closest_error) + next_idx

    if target_idx >= profile.last_idx:  # TODO improve movement to final index
        target_idx = profile.last_idx

    print('target_idx: ' + str(target_idx))
    # Project RMS error onto front axle vector
    # front_axle_vec = [robot.Dx, robot.Dy]
    # error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx


def pid_control(target, current):
    return Kp * (target - current)


def face_initial_yaw():
    initial_yaw = profile.yaw[0]
    d_yaw = robot.yaw - initial_yaw
    print(d_yaw)
    while abs(robot.yaw - initial_yaw) > initial_yaw_delta:
        if robot.yaw - initial_yaw < 0:
            robot.drive(sweeping_angle, 0)
        else:
            robot.drive(-1*sweeping_angle, 0)


def preform_motion_profile():
    face_initial_yaw()
    target_idx = calc_target_index()

    reached_destination = (abs(robot.x - profile.X[-1]) < final_position_delta) and (abs(robot.y - profile.Y[-1]) < final_position_delta)
    flag = check_position(0) and (profile.last_idx > target_idx) and not reached_destination

    while flag:
        # a = pid_control(profile.v[target_idx], robot.v)
        delta_yaw, target_idx = stanley_control(target_idx)
        robot.drive(15*delta_yaw, coursing_velocity)

        reached_destination = (abs(robot.x - profile.X[-1]) < final_position_delta) and (abs(robot.y - profile.Y[-1]) < final_position_delta)
        flag = check_position(0) and (profile.last_idx > target_idx) and not reached_destination

        plt.plot(robot.X, robot.Y, label="course")
        plt.plot(profile.X, profile.Y, "r", label="spline")
        plt.plot(profile.X, profile.Y, ".b", label="spline")
        plt.legend()
        plt.title('actual X-Y coordinates')
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)
        plt.show()

    if reached_destination:
        return 0
    elif (not check_position(0)) or (profile.last_idx <= target_idx):
        return 1


def check_position(new_obstacle_mode):
    # handling new obstacle.
    at0, at45L, at45R = facing_new_obstacle()
    if (at0 or at45L or at45L) and (not new_obstacle_mode):
        robot.drive(0, stop_velocity)
        pass_obstacle()
        return 0

    if (at0 == 0 or at45L == 0 or at45L == 0) and new_obstacle_mode:
        robot.drive(0, stop_velocity)
        return 0

    # handling the situation the we are too close to an obstacle.
    if are_we_too_close():
        robot.drive(0, stop_velocity)
        return 0

    return 1


def are_we_too_close():
    return (robot.Obs0 <= Closeness0 and robot.Obs0 != -1) or \
           (robot.Obs45L <= Closeness45 and robot.Obs45L != -1) or \
           (robot.Obs45R <= Closeness45 and robot.Obs45R != -1)


def facing_new_obstacle():
    at0 = at45L = at45R = 0

    if robot.Obs0 != 0:
        at0 = check_obstacle(robot.x, robot.y, robot.yaw, robot.Obs0, 0)
    if robot.Obs45L != 0:
        at45L = check_obstacle(robot.x, robot.y, robot.yaw, robot.Obs45L, 45)
    if robot.Obs45R != 0:
        at45R = check_obstacle(robot.x, robot.y, robot.yaw, robot.Obs45R, -45)

    return at0, at45L, at45R


def check_obstacle(robot_x, robot_y, robot_angle, obs_dis, obs_angle):
    obs_x = robot_x + obs_dis*np.cos(robot_angle + obs_angle)
    obs_y = robot_y + obs_dis*np.sin(robot_angle + obs_angle)
    return 0  # TODO add peleg function to map a new obstacle


def pass_obstacle():
    position_the_robot_at_90_degree_to_obstacle()
    drive_parallel_to_obstacle()

    robot.drive(0, coursing_velocity)
    robot.drive(90, stop_velocity)
    robot.drive(0, coursing_velocity)


def drive_parallel_to_obstacle():
    start_passing_time = time()
    dt = time() - start_passing_time
    while robot.Obs45L != 0 and check_position(1) and (dt < pass_obstacle_timeout):
        robot.drive(0, coursing_velocity)
        position_the_robot_at_90_degree_to_obstacle()
        dt = time() - start_passing_time


def position_the_robot_at_90_degree_to_obstacle():
    while robot.Obs0 != 0:
        robot.drive(sweeping_angle, 0)
        # TODO add peleg function to map a new obstacle


def recalculate_route(x_int, y_int):
    # x = [0.0, 60.0, 120.0, 60]
    # y = [0.0, -60.0, 0.0, 60]

    x = [0.0, 50.0, 100.0, 150.0, 200]
    y = [0.0, -25.0, 0, 25.0, 0.0]

    x = [xi + x_int for xi in x]
    y = [yi + y_int for yi in y]

    X, Y, yaw, ck, s = create_motion_profile(x, y)
    v = create_target_velocity_profile(ck, s)
    profile.update_profile(X, Y, yaw, v, s)
    profile.plot_motion_profile(x, y)


def initialize_motion(x, y):
    X, Y, yaw, ck, s = create_motion_profile(x, y)
    v = create_target_velocity_profile(ck, s)
    profile.update_profile(X, Y, yaw, v, s)
    profile.plot_motion_profile(x, y)


def map_robot_wheels_commends_to_yaw():
    yaw_map = np.array([0, 0, 0, 0, 0])
    for i in range(300, 700, 10):
        for j in range(-300, -700, -10):
            robot.state.drive(int(i), int(j))
            sleep(1)
            robot.update_params()
            print(i, j, robot.Dx, robot.Dy, robot.yaw)
            yaw_map = np.vstack((yaw_map, np.array([i, j, robot.Dx, robot.Dy, robot.yaw])))
    return yaw_map


def map_robot_wheels_commends_to_speed():
    yaw_map = np.array([0, 0, 0, 0, 0])
    for i in range(200, 500, 5):
            robot.state.drive(int(i), int(i))
            sleep(1)
            robot.update_params()
            yaw_map = np.vstack((yaw_map, np.array([i, i, robot.Dx, robot.Dy, robot.yaw])))
    return yaw_map


def brakes_test():
    const = -1000
    stop = 300 * np.sign(const) * -1

    robot.drive(0, const)
    print(robot.Obs0, robot.Obs45L, robot.Obs45R, robot.v)
    if not check_position(0):
        print("stop!")
        return
    robot.drive(0, const)
    print(robot.Obs0, robot.Obs45L, robot.Obs45R, robot.v)
    if not check_position(0):
        print("stop!")
        return
    robot.drive(0, const)
    print(robot.Obs0, robot.Obs45L, robot.Obs45R, robot.v)
    if not check_position(0):
        print("stop!")
        return
    robot.drive(0, const)
    print(robot.Obs0, robot.Obs45L, robot.Obs45R, robot.v)
    if not check_position(0):
        print("stop!")
        return
    robot.drive(0, const)
    print(robot.Obs0, robot.Obs45L, robot.Obs45R, robot.v)
    if not check_position(0):
        print("stop!")
        return
    robot.drive(0, const)
    print(robot.Obs0, robot.Obs45L, robot.Obs45R, robot.v)
    if not check_position(0):
        print("stop!")
        return
    robot.drive(0, stop)
    print(robot.Obs0, robot.Obs45L, robot.Obs45R, robot.v)

    # robot.drive(0, const)
    # robot.drive(0, const)
    # robot.drive(0, const)
    # robot.drive(0, const)
    # robot.drive(0, const)
    # robot.drive(0, const)
    # robot.drive(0, stop)

def main():
    brakes_test()
    print(robot.Obs0, robot.Obs45L, robot.Obs45R, robot.v)

    # yaw_map = map_robot_wheels_commends_to_yaw()
    # np.savetxt("yaw_map.csv", yaw_map, delimiter=",")
    # speed_map = map_robot_wheels_commends_to_speed()
    # np.savetxt("speed_map.csv", speed_map, delimiter=",")

    # recalculate_route(robot.x, robot.y)
    #
    # while preform_motion_profile():
    #     recalculate_route(robot.x, robot.y)
    #
    # robot.plot_actual_motion()
    # print('FINISHED!!!')
    robot.state.terminate()
    return 0


if __name__ == '__main__':
    main()
