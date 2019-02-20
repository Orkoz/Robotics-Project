from Robot_Control import *
from alg import *


# world_gx = -130 # [cm]
# world_gy = -155  # [cm]


# obs_vec_x = [-30, -30, -30, -30, -30, -30]
# obs_vec_y = [-160, -150, -140, -130, -120]

# world_gx = 0  # [cm]
# world_gy = 0  # [cm]
# obs_vec_x = [-65, -55, -45, -35, -25, -15, -5,   5,   10,  15,  25,  35,  45,  55,  65, -65, -55, -45, -35, -25, -15, -5,   5,   10,  15,  25,  35,  45,  55,  65]
# obs_vec_y = [-62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -62, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72, -72]

world_gx = 1  # [cm]
world_gy = 0  # [cm]
obs_vec_x = []
obs_vec_y = []

def main():
    create_map()
    x, y, yaw_mat = calculate_route(robot.x, robot.y)
    print(robot.x, robot.y)
    if not len(x):
        print('There is no Route')
        return 0
    # print_arrow_yaw_mat(yaw_mat)
    # print_node_mat()
    p = print_node_map(robot.x, robot.y)
    p.show()

    initialize_motion(x, y, yaw_mat)

    while preform_motion_profile():
        x, y, yaw_mat = calculate_route(robot.x, robot.y)
        if not len(x):
            print('There is no Route')
            return 0
        initialize_motion(x, y, yaw_mat)
        p = print_node_map(robot.x, robot.y)
        p.show()

    # robot.plot_actual_motion()
    # p = print_node_map(robot.x, robot.y)
    # p.show()

    print(robot.x, robot.y)
    print('FINISHED!!!')
    robot.state.terminate()
    return 0


if __name__ == '__main__':
    main()