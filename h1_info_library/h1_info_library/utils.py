# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

def determine_coeff_and_mode(index_of_joint_of_unitree_h1: int) -> tuple:
    """Determine motor coefficients and mode by joint index.

    This function determines the proportional (Kp) and derivative (Kd)
    coefficients and operation mode for motors based on the absolute
    joint index of the Unitree H1 robot.

    Args:
        index_of_joint_of_unitree_h1 (int): Absolute index of the joint
            in the Unitree H1 robot.

    Returns:
        tuple: A tuple containing (Kp, Kd, mode) where:
            - Kp (float): Proportional coefficient
            - Kd (float): Derivative coefficient
            - mode (int): Operation mode for motor enable/disable control

    Note:
        Different joint sizes use different coefficient values:
        - Size S joints (indices 10, 11): Kp=80.0, Kd=2.0
        - Size L joints (indices 2, 5): Kp=200.0, Kd=5.0
        - Size XS joints (indices 32, 33): Kp=10.0, Kd=2.0
        - Finger joints (indices 20-31): Kp=0.0, Kd=0.0
        - All other joints: Kp=100.0, Kd=3.0

        Mode values:
        - Indices < 9: mode = 0x0A
        - Indices > 9: mode = 0x01
        - Index == 9: mode = 0x00
    """
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
        Kp = 20.0
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
