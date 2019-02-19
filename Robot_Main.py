from Robot_Control import *
from alg import *


def main():
    create_map()
    print(robot.x, robot.y)
    x, y, yaw_mat = calculate_route(robot.x, robot.y)
    # print_arrow_yaw_mat(yaw_mat)
    # print_node_mat()
    initialize_motion(x, y, yaw_mat)

    while preform_motion_profile():
        x, y, yaw_mat = calculate_route(robot.x, robot.y)
        initialize_motion(x, y, yaw_mat)

    # robot.plot_actual_motion()
    print(robot.x, robot.y)
    print('FINISHED!!!')
    robot.state.terminate()
    return 0


if __name__ == '__main__':
    main()