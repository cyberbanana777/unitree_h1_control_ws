def determine_coeff_and_mode(index_of_joint_of_unitree_h1: int) -> tuple:
    '''
    Function for determining the coefficients Kp, Kd and mode for motors
    by absolute index
    '''
    size_S = [10, 11]
    size_L = [2, 5]
    fingers = [x for x in range(20, 32)]
    size_XS = [x for x in range(32, 34)]

    # determine Kp and Kd
    if index_of_joint_of_unitree_h1 in size_S:
        Kp = 80.0
        Kd = 2.0

    elif index_of_joint_of_unitree_h1 in size_L:
        Kp = 200.0
        Kd = 5.0

    elif index_of_joint_of_unitree_h1 in size_XS:
        Kp = 10.0
        Kd = 2.0

    elif index_of_joint_of_unitree_h1 in fingers:
        Kp = 0.0
        Kd = 0.0

    else:
        Kp = 100.0
        Kd = 3.0

    # determine mode for enable
    if index_of_joint_of_unitree_h1 < 9:
        mode = 0x0A
    elif index_of_joint_of_unitree_h1 > 9:
        mode = 0x01
    else:
        mode = 0x00

    return (Kp, Kd, mode)
