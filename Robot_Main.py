from Robot_Control import *
from alg import *
import threading

# min_x = -200     # [cm]
# max_x = 200  # [cm]
# min_y = -260  # [cm]
# max_y = 50   # [cm]
# world_gx = -130 # [cm]
# world_gy = -155  # [cm]
# obs_vec_x = [-80, -70, -60, -50, -40, -30, -20, -10,   0,  10,  20,  30,  40,  50,  60, -80, -70, -60, -50, -40, -30, -20, -10,   0,  10,  20,  30,  40,  50,  60]
# obs_vec_y = [-70, -70, -70, -70, -70, -70, -70, -70, -70, -70, -70, -70, -70, -70, -70, -80, -80, -80, -80, -80, -80, -80, -80, -80, -80, -80, -80, -80, -80, -80]

# world_gx = 0  # [cm]
# world_gy = 0  # [cm]
# obs_vec_x = [-65, -55, -45, -35, -25, -15, -5,   5,   10,  15,  25,  35,  45,  55,  65, -65, -55, -45, -35, -25, -15, -5,   5,   10,  15,  25,  35,  45,  55,  65]
# obs_vec_y = [-62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72]

world_gx = 0  # [cm]
world_gy = 0  # [cm]
obs_vec_x = []
obs_vec_y = []


def main():
    print(robot.x, robot.y)
    create_map()
    x, y, yaw_mat = calculate_route(robot.x, robot.y)
    print(robot.x, robot.y)
    if not len(x):
        print('There is no Route')
        return 0

    t = threading.Thread(target=print_node_map, args=(robot.x, robot.y, ))
    t.start()

    initialize_motion(x, y, yaw_mat)

    while preform_motion_profile():
        x, y, yaw_mat = calculate_route(robot.x, robot.y)
        if not len(x):
            print('There is no Route')
            return 0
        initialize_motion(x, y, yaw_mat)
        print('NEW Route')
        t = threading.Thread(target=print_node_map, args=(robot.x, robot.y,))
        t.start()

    print(robot.x, robot.y)
    print('FINISHED!!!')
    # robot.plot_actual_motion()
    robot.state.terminate()
    return 0


if __name__ == '__main__':
    main()