from Rclient import RClient
import numpy as np
from time import time
from time import sleep
import matplotlib.pyplot as plt
from alg import world_to_map
from alg import distance_to_goal
from alg import check_new_obstacle
import threading
from alg import front_or_back_obstacle
from alg import min_x,max_x,min_y,max_y


# Variables
Closeness0 = 40.0  # [cm]
Closeness45 = 40.0  # [cm]
Edge_Closeness = 40.0  # [cm]
Kp = 3.0  # speed proportional gain
initial_yaw_delta = 10.0  # [deg]
go_around_obstacle_delta_yaw = 90  # [deg]
critical_yaw = 30  # [deg]
final_position_delta = 10.0  # [cm]
drive_directly_to_target_d = 2  # [factor]
sweeping_angle = 280.0  # [cm]
coursing_velocity = 400.0  # [wheel power]
drive_directly_to_target_velocity = 300  # [wheel power]
stop_velocity = -100  # [wheel power]
sleep_time = 0.5  # [sec]
pass_obstacle_timeout = 10  # [sec]



class Robot(object):
    def __init__(self):
        super(Robot, self).__init__()
        self.state = RClient("192.168.1.157", 2777)
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
        self.dt = 0.0000001
        self.time_stamp = time()
        self.X = []
        self.Y = []
        self.YAW = []
        self.V = []
        self.T = [0]
        self.csv_file = np.array([self.x, self.y, self.Dx, self.Dy, self.yaw, self.v, self.dt])
        self.e = threading.Event()
        self.update_params_thread = threading.Thread(target=self.update_params)
        self.update_params_thread.start()

    def update_params(self):
        while 1:
            x, y, self.Dx, self.Dy, self.Obs45R, self.Obs0, self.Obs45L = self.state.sense()
            if abs(x) < 1000:
                self.yaw = np.rad2deg(np.arctan2(self.Dy, self.Dx))
                self.dt = time() - self.time_stamp
                self.time_stamp = time()
                # self.v = np.sqrt((x - self.x)**2 + (y - self.y)**2) / self.dt
                self.x = x
                self.y = y
                self.X.append(self.x)
                self.Y.append(self.y)
                self.YAW.append(self.yaw)
                self.V.append(self.v)
                self.T.append(self.T[-1] + self.dt)
                self.csv_file = np.vstack((self.csv_file, np.array([self.x, self.y, self.Dx, self.Dy, self.yaw, self.v, self.dt])))
            self.e.set()

    def drive(self, delta_yaw, v):
        left, right = convert_angle_and_velocity_to_wheels_commends(delta_yaw, v)
        self.state.drive(int(left), int(right))
        self.e.clear()
        self.e.wait()

    def plot_actual_motion(self):
        plt.plot(self.X, self.Y, label="course")
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
        self.yaw_mat = []

    def update_profile(self, X, Y, yaw_mat):
        self.X = X
        self.Y = Y
        self.yaw_mat = yaw_mat


robot = Robot()
profile = MotionProfile()


def convert_angle_and_velocity_to_wheels_commends(delta_yaw, v):
    left = v + delta_yaw
    right = v - delta_yaw

    if np.abs(left) > 1000:  # TODO add condition on 0
        left = np.sign(left)*1000
    if np.abs(right) > 1000:
        right = np.sign(right)*1000
    return left, right


def stanley_control():

    x_map, y_map = world_to_map(robot.x, robot.y)
    theta_d = (np.rad2deg(profile.yaw_mat[x_map, y_map]) - robot.yaw)
    if theta_d > critical_yaw:
        face_yaw(np.rad2deg(profile.yaw_mat[x_map, y_map]))
        theta_d = (np.rad2deg(profile.yaw_mat[x_map, y_map]) - robot.yaw)

    return theta_d*Kp


def face_yaw(initial_yaw):
    d_yaw = robot.yaw - initial_yaw
    while abs(d_yaw) > initial_yaw_delta:
        facing_new_obstacle()
        #
        # if distance_to_goal(robot.x, robot.y) < final_position_delta ** 2: # reached_destination
        #     return 1

        if abs(d_yaw) <= (360 - abs(d_yaw)):
            if d_yaw <= 0:
                robot.drive(sweeping_angle, 0)
            else:
                robot.drive(-1*sweeping_angle, 0)
        else:
            if d_yaw >= 0:
                robot.drive(sweeping_angle, 0)
            else:
                robot.drive(-1*sweeping_angle, 0)
        d_yaw = robot.yaw - initial_yaw


def preform_motion_profile():
    print('start motion profile')
    x_map, y_map = world_to_map(robot.x, robot.y)

    face_yaw(np.rad2deg(profile.yaw_mat[x_map, y_map]))
    delta_yaw = stanley_control()
    d = distance_to_goal(robot.x, robot.y)
    reached_destination = d < final_position_delta ** 2
    check_position_flag = check_position(0)
    flag = check_position_flag and not reached_destination

    while flag:
        robot.drive(delta_yaw, coursing_velocity)

        delta_yaw = stanley_control()
        check_position_flag = check_position(0)
        d = distance_to_goal(robot.x, robot.y)
        reached_destination = d < drive_directly_to_target_d*(final_position_delta**2)
        # reached_destination = d < (final_position_delta**2)
        flag = check_position_flag and not reached_destination

    t = threading.Thread(target=live_plot())
    t.start()

    if reached_destination:
        return drive_directly_to_target()
    elif not check_position_flag:
        return 1


def live_plot():
    plt.plot(robot.X, robot.Y, label="course")
    plt.plot(profile.X, profile.Y, "r", label="spline")
    plt.legend()
    plt.title('actual X-Y coordinates')
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    plt.grid(True)
    plt.show()


def drive_directly_to_target():
    print('driving directly to target')

    # robot_target_angle = np.rad2deg(np.arctan2((world_gy - robot.y), (world_gx - robot.x)))
    # face_yaw(robot_target_angle)
    x_map, y_map = world_to_map(robot.x, robot.y)
    face_yaw(np.rad2deg(profile.yaw_mat[x_map, y_map]))

    d = distance_to_goal(robot.x, robot.y)
    reached_destination = d < final_position_delta ** 2

    while not reached_destination:
        robot.drive(0, drive_directly_to_target_velocity)
        d = distance_to_goal(robot.x, robot.y)
        reached_destination = d < final_position_delta**2

        # robot_target_angle = np.rad2deg(np.arctan2((world_gy - robot.y), (world_gx - robot.x)))
        # face_yaw(robot_target_angle)
        x_map, y_map = world_to_map(robot.x, robot.y)
        face_yaw(np.rad2deg(profile.yaw_mat[x_map, y_map]))

    return 0


def check_position(new_obstacle_mode):
    if are_we_too_close_to_edge():
        robot.drive(0, stop_velocity)

    at0, at45L, at45R = facing_new_obstacle()
    # handling the situation the we are too close to an obstacle.
    if are_we_too_close_to_obstacle() and (at45L == 1 or at45R == 1):
        robot.drive(0, stop_velocity)
        robot.drive(0, -500)
        print('we are too close!')
        return 0

    # handling new obstacle.
    if (at0 == 1 or at45L == 1 or at45L == 1) and (not new_obstacle_mode):
        robot.drive(0, stop_velocity)
        print('passing obstacle!')
        pass_obstacle()
        return 0

    if (at0 == 0 and at45L == 0 and at45R == 0) and new_obstacle_mode:
        robot.drive(0, stop_velocity)
        return 0

    # if (at0 == 2 or at45L == 2 or at45R == 2) and new_obstacle_mode:
    #     robot.drive(0, stop_velocity)
    #     return 0

    return 1


def are_we_too_close_to_obstacle():
    return (robot.Obs0 <= Closeness0 and robot.Obs0 != -1) or \
           (robot.Obs45L <= Closeness45 and robot.Obs45L != -1) or \
           (robot.Obs45R <= Closeness45 and robot.Obs45R != -1)


def are_we_too_close_to_edge():
    dis_to_edge_x = min(abs(robot.x - min_x), abs(robot.x - max_x))
    dis_to_edge_y = min(abs(robot.y - min_y), abs(robot.y - max_y))
    if (dis_to_edge_x/robot.Dx) <= Edge_Closeness or (dis_to_edge_y/robot.Dy) <= Edge_Closeness:
        return 1

    return 0


def facing_new_obstacle():
    at0 = at45L = at45R = 0  # no obstacle

    if robot.Obs0 != -1:
        if check_obstacle(robot.x, robot.y, robot.yaw, robot.Obs0, 0):
            at0 = 1  # new obstacle
        else:
            at0 = 2  # old obstacle
    if robot.Obs45L != -1:
        if check_obstacle(robot.x, robot.y, robot.yaw, robot.Obs45L, -45):
            at45L = 1
        else:
            at45L = 2
    if robot.Obs45R != -1:
        if check_obstacle(robot.x, robot.y, robot.yaw, robot.Obs45R, 45):
            at45R = 1
        else:
            at45R = 2

    return at0, at45L, at45R


def check_obstacle(robot_x, robot_y, robot_angle, obs_dis, obs_angle):
    obs_x = robot_x + obs_dis*np.cos(np.deg2rad(robot_angle + obs_angle))
    obs_y = robot_y + obs_dis*np.sin(np.deg2rad(robot_angle + obs_angle))
    ans = check_new_obstacle(obs_x, obs_y)

    if ans == 1 and front_or_back_obstacle(robot_x, robot_y, obs_x, obs_y):
        ans = 0

    return ans


def pass_obstacle():
    position_the_robot_at_90_degree_to_obstacle()
    drive_parallel_to_obstacle()
    go_around_obstacle()


def position_the_robot_at_90_degree_to_obstacle():
    while robot.Obs0 != -1:
        robot.drive(sweeping_angle, 0)
        facing_new_obstacle()


def drive_parallel_to_obstacle():
    start_passing_time = time()
    dt = time() - start_passing_time
    while robot.Obs45L != -1 and (dt < pass_obstacle_timeout) and check_position(1):
        robot.drive(0, coursing_velocity)
        position_the_robot_at_90_degree_to_obstacle()
        dt = time() - start_passing_time


def go_around_obstacle():
    initial_yaw = robot.yaw
    while robot.yaw != initial_yaw - go_around_obstacle_delta_yaw and check_position(1):
        robot.drive(0, coursing_velocity)
        while robot.Obs45L != -1:
            robot.drive(-1*sweeping_angle, 0)
            facing_new_obstacle()


def initialize_motion(x, y, yaw_mat):
    profile.update_profile(x, y, yaw_mat)


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

    robot.state.terminate()
    return 0


if __name__ == '__main__':
    main()
