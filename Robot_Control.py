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
Closeness0 = 30.0  # [cm]
Closeness45 = 30.0  # [cm]
Edge_Closeness = 30.0  # [cm]
Kp = 3.0  # speed proportional gain
initial_yaw_delta = 10.0  # [deg]
go_around_obstacle_delta_yaw = 45  # [deg]
movement_delta_yaw = 1  # [deg]
movement_delta_x_y = 1  # [deg]
critical_yaw = 30  # [deg]
final_position_delta = 10.0  # [cm]
drive_directly_to_target_d = 2  # [factor]
sweeping_angle = 310.0  # [cm]
coursing_velocity = 270.0  # [wheel power]
drive_directly_to_target_velocity = 230.0  # [wheel power]
stop_velocity = -300  # [wheel power]
sleep_time = 0.2  # [sec]
pass_obstacle_timeout = 1000  # [sec]
to_close_to_edge_flag = False


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
        # self.V = []
        self.T = [0]
        self.csv_file = np.array([self.x, self.y, self.Dx, self.Dy, self.yaw, self.dt])
        self.e = threading.Event()
        self.update_params_thread = threading.Thread(target=self.update_params)
        self.update_params_thread.start()
        self.start_motion_sweeping_angle = 0  # [cm]
        self.start_motion_coursing_velocity = 0  # [wheel power]
        self.to_close_to_edge_flag = self.are_we_too_close_to_edge()

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
                # self.V.append(self.v)
                self.T.append(self.T[-1] + self.dt)
                self.csv_file = np.vstack((self.csv_file, np.array([self.x, self.y, self.Dx, self.Dy, self.yaw, self.dt])))
            self.e.set()

    def drive(self, delta_yaw, v):
        prev_x = round(robot.x)
        prev_y = round(robot.y)
        prev_yaw = round(robot.yaw)

        left, right = convert_angle_and_velocity_to_wheels_commends(delta_yaw, v)
        self.state.drive(int(left), int(right))
        sleep(sleep_time)
        self.e.clear()
        self.e.wait()

        # if v == 0 and abs(round(robot.yaw) - prev_yaw) < movement_delta_yaw:
        #     robot.start_rotational_motion(delta_yaw, prev_yaw)
        # if delta_yaw == 0 and abs(round(robot.x) - prev_x) < movement_delta_x_y and abs(round(robot.y) - prev_y) < movement_delta_x_y:
        #     robot.start_linear_motion(v, prev_x, prev_y)
        if abs(round(robot.yaw) - prev_yaw) < movement_delta_yaw and abs(round(robot.x) - prev_x) < movement_delta_x_y and abs(round(robot.y) - prev_y) < movement_delta_x_y:
            robot.start_motion(delta_yaw, prev_yaw, v, prev_x, prev_y)

    def start_motion(self, delta_yaw, prev_yaw, v, prev_x, prev_y):
        while abs(round(robot.yaw) - prev_yaw) < movement_delta_yaw and abs(round(robot.x) - prev_x) < movement_delta_x_y and abs(round(robot.y) - prev_y) < movement_delta_x_y:
            self.start_motion_sweeping_angle = min(self.start_motion_sweeping_angle + 1, 50)
            self.start_motion_sweeping_angle = min(self.start_motion_sweeping_angle + 1, 50)
            left, right = convert_angle_and_velocity_to_wheels_commends(delta_yaw + self.start_motion_sweeping_angle, v + self.start_motion_coursing_velocity)
            self.state.drive(int(left), int(right))
            self.e.clear()
            self.e.wait()
        self.start_motion_sweeping_angle = 0
        self.start_motion_coursing_velocity = 0

    def start_rotational_motion(self, delta_yaw, prev_yaw):
        while abs(round(robot.yaw) - prev_yaw) < movement_delta_yaw:
            self.start_motion_sweeping_angle = self.start_motion_sweeping_angle + 1
            print(self.start_motion_sweeping_angle)
            left, right = convert_angle_and_velocity_to_wheels_commends(delta_yaw + self.start_motion_sweeping_angle, 0)
            self.state.drive(int(left), int(right))
            self.e.clear()
            self.e.wait()
        self.start_motion_sweeping_angle = 0

    def start_linear_motion(self, v, prev_x, prev_y):
        while abs(round(robot.x) - prev_x) < movement_delta_x_y and abs(round(robot.y) - prev_y) < movement_delta_x_y:
            self.start_motion_coursing_velocity = self.start_motion_coursing_velocity + 1
            print(self.start_motion_coursing_velocity)
            left, right = convert_angle_and_velocity_to_wheels_commends(0, v + self.start_motion_coursing_velocity)
            self.state.drive(int(left), int(right))
            self.e.clear()
            self.e.wait()
        self.start_motion_coursing_velocity = 0

    def plot_actual_motion(self):
        plt.plot(self.X, self.Y, label="course")
        plt.legend()
        plt.title('actual X-Y coordinates')
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)
        plt.show()

        plt.plot(self.T, self.YAW)
        plt.title('actual yaw')
        plt.xlabel("Time[s]")
        plt.ylabel("Yaw[deg]")
        plt.grid(True)
        plt.show()

        np.savetxt("actual motion/robot -" + str(time()) + ".csv", self.csv_file, delimiter=",")

    def are_we_too_close_to_edge(self):
        dis_to_min_edge_x = abs(self.x - min_x)
        dis_to__min_edge_y = abs(self.y - min_y)
        dis_to_max_edge_x =  abs(self.x - max_x)
        dis_to__max_edge_y = abs(self.y - max_y)

        a = Edge_Closeness*self.Dx
        b = Edge_Closeness*self.Dy

        if self.Dx < 0 and dis_to_min_edge_x <= abs(Edge_Closeness*self.Dx):
            return 1
        if self.Dx >= 0 and dis_to_max_edge_x <= Edge_Closeness*self.Dx:
            return 1
        if self.Dy < 0 and dis_to__min_edge_y <= abs(Edge_Closeness*self.Dy):
            return 1
        if self.Dy >= 0 and dis_to__max_edge_y <= Edge_Closeness*self.Dy:
            return 1

        return 0

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

    if np.abs(left) > 1000:
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
    if initial_yaw == 180 or initial_yaw == -180:
        print
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

    # t = threading.Thread(target=live_plot())
    # t.start()

    if reached_destination:
        return drive_directly_to_target()
    elif not check_position_flag:
        return 1


def live_plot():
    m = len(robot.X)
    plt.plot(robot.X[0:m], robot.Y[0:m], label="course")
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
        if not robot.to_close_to_edge_flag:
            print('we are too close to edge!')
            robot.drive(0, stop_velocity)
            robot.to_close_to_edge_flag = True
        if new_obstacle_mode:
            return 0
    else:
        # print('we are no longer close to edge!')
        robot.to_close_to_edge_flag = False

    at0, at45L, at45R = facing_new_obstacle()
    # handling the situation the we are too close to an obstacle.
    if are_we_too_close_to_obstacle() and (at45L == 1 or at45L == 1 or at45R == 1):
        robot.drive(0, stop_velocity)
        robot.drive(0, -500)
        print('we are too close to obstacle!')
        return 0

    # handling new obstacle.
    if (at0 == 1 or at45L == 1 or at45R == 1) and (not new_obstacle_mode):
        robot.drive(0, stop_velocity)
        print('passing obstacle!')
        pass_obstacle()
        return 0

    # if (at0 == 0 and at45L == 0 and at45R == 0) and new_obstacle_mode:
    #     robot.drive(0, stop_velocity)
    #     return 0

    # if (at0 == 2 or at45L == 2 or at45R == 2) and new_obstacle_mode:
    #     robot.drive(0, stop_velocity)
    #     return 0

    return 1


def are_we_too_close_to_obstacle():
    return (robot.Obs0 <= Closeness0 and robot.Obs0 != -1) or \
           (robot.Obs45L <= Closeness45 and robot.Obs45L != -1) or \
           (robot.Obs45R <= Closeness45 and robot.Obs45R != -1)


def are_we_too_close_to_edge():
    dis_to_min_edge_x = abs(robot.x - min_x)
    dis_to__min_edge_y = abs(robot.y - min_y)
    dis_to_max_edge_x =  abs(robot.x - max_x)
    dis_to__max_edge_y = abs(robot.y - max_y)

    a = Edge_Closeness*robot.Dx
    b = Edge_Closeness*robot.Dy

    if robot.Dx < 0 and dis_to_min_edge_x <= abs(Edge_Closeness*robot.Dx):
        return 1
    if robot.Dx >= 0 and dis_to_max_edge_x <= Edge_Closeness*robot.Dx:
        return 1
    if robot.Dy < 0 and dis_to__min_edge_y <= abs(Edge_Closeness*robot.Dy):
        return 1
    if robot.Dy >= 0 and dis_to__max_edge_y <= Edge_Closeness*robot.Dy:
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
    robot.drive(0, coursing_velocity)
    robot.drive(0, coursing_velocity)


def position_the_robot_at_90_degree_to_obstacle():
    while robot.Obs0 != -1:
        robot.drive(sweeping_angle, 0)
        facing_new_obstacle()
        # sleep(sleep_time*2)


def drive_parallel_to_obstacle():
    start_passing_time = time()
    dt = time() - start_passing_time
    while robot.Obs45L != -1 and (dt < pass_obstacle_timeout) and check_position(1):
        robot.drive(0, coursing_velocity)
        position_the_robot_at_90_degree_to_obstacle()
        dt = time() - start_passing_time


def go_around():
    initial_yaw = robot.yaw
    delta_yaw = robot.yaw - (initial_yaw - go_around_obstacle_delta_yaw)
    while delta_yaw > initial_yaw_delta and check_position(1):
        robot.drive(0, coursing_velocity)
        while robot.Obs45L == -1:
            robot.drive(-1*sweeping_angle, 0)
            facing_new_obstacle()
            # sleep(sleep_time * 2)
        robot.drive(sweeping_angle, 0)


def initialize_motion(x, y, yaw_mat):
    profile.update_profile(x, y, yaw_mat)


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
