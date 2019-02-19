from Robot_Control import *


def main():
    min_x = 0.0     # [cm]
    max_x = 50.0    # [cm]
    min_y = -25.0   # [cm]
    max_y = 25.0    # [cm]
    world_gx = 43.0  # [cm]
    world_gy = 15.0  # [cm]
    world_sx = 0.0
    world_sy = -15.0
    reso = 5        # [cm]

    world_map = create_map(min_x, max_x, min_y, max_y, world_gx, world_gy, world_sx, world_sy, reso, Q_obs)
    x, y, angle_map = calculate_route(world_map, robot.x, robot.y)

    initialize_motion(x, y)

    while preform_motion_profile():
        x, y, angle_map = calculate_route(world_map, robot.x, robot.y)
        initialize_motion(x, y)

    robot.plot_actual_motion()
    print('FINISHED!!!')
    robot.state.terminate()
    return 0


if __name__ == '__main__':
    main()