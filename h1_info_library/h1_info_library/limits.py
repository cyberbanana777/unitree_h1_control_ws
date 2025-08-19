# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

LIMITS_OF_JOINTS_WITH_HANDS_FROM_VENDOR = {
    0: (-0.43, 0.43),  # right_hip_roll_joint
    1: (-3.14, 2.53),  # right_hip_pitch_joint
    2: (-0.26, 2.05),  # right_knee_joint
    3: (-0.43, 0.43),  # left_hip_roll_joint
    4: (-3.14, 2.53),  # left_hip_pitch_joint
    5: (-0.26, 2.05),  # left_knee_joint
    6: (-2.35, 2.35),  # torso_joint
    7: (-0.43, 0.43),  # left_hip_yaw_joint
    8: (-0.43, 0.43),  # right_hip_yaw_joint
    9: (0.0, 1.0),  # IMPACT
    10: (-0.87, 0.52),  # left_ankle_joint
    11: (-0.87, 0.52),  # right_ankle_joint
    12: (-2.87, 2.87),  # right_shoulder_pitch_joint
    13: (-3.11, 0.34),  # right_shoulder_roll_joint
    14: (-4.45, 1.3),  # right_shoulder_yaw_joint
    15: (-1.25, 2.61),  # right_elbow_joint
    16: (-2.87, 2.87),  # left_shoulder_pitch_joint
    17: (-0.34, 3.11),  # left_shoulder_roll_joint
    18: (-1.3, 4.45),  # left_shoulder_yaw_joint
    19: (-1.25, 2.61),  # left_elbow_joint
    20: (0.0, 1.0),  # right_pinky
    21: (0.0, 1.0),  # right_ring
    22: (0.0, 1.0),  # right_middle
    23: (0.0, 1.0),  # right_index
    24: (0.0, 1.0),  # right_thumb_bend
    25: (0.0, 1.0),  # right_thumb_rotation
    26: (0.0, 1.0),  # left_pinky
    27: (0.0, 1.0),  # left_ring
    28: (0.0, 1.0),  # left_middle
    29: (0.0, 1.0),  # left_index
    30: (0.0, 1.0),  # left_thumb_bend
    31: (0.0, 1.0),  # left_thumb_rotation
    32: (-1.1, 4.58),  # left_wrist
    33: (-6.0, -0.23),  # right_wrist
}


LIMITS_OF_JOINTS_WITH_HANDS_FOR_TELEOPERATION = {
    0: (-0.43, 0.43),  # right_hip_roll_joint M
    1: (-3.14, 2.53),  # right_hip_pitch_joint M
    2: (-0.26, 2.05),  # right_knee_joint L
    3: (-0.43, 0.43),  # left_hip_roll_joint M
    4: (-3.14, 2.53),  # left_hip_pitch_joint M
    5: (0.26, 2.05),  # left_knee_joint L
    6: (-2.35, 2.35),  # torso_joint M
    7: (-0.43, 0.43),  # left_hip_yaw_joint M
    8: (-1.3, 1.5),  # right_hip_yaw_joint M
    9: (0.0, 1.0),  # IMPACT
    10: (-0.87, 0.52),  # left_ankle_joint S
    11: (-0.87, 0.52),  # right_ankle_joint S
    12: (-1.9, 0.5),  # right_shoulder_pitch_joint M
    13: (-2.2, 0.0),  # right_shoulder_roll_joint M
    14: (-1.5, 1.3),  # right_shoulder_yaw_joint M
    15: (-1.1, 1.65),  # right_elbow_joint M
    16: (-1.9, 0.5),  # left_shoulder_pitch_joint M
    17: (0.0, 2.2),  # left_shoulder_roll_joint M
    18: (-1.3, 1.5),  # left_shoulder_yaw_joint M
    19: (-1.1, 1.65),  # left_elbow_joint M
    20: (0.0, 1.0),  # right_pinky
    21: (0.0, 1.0),  # right_ring
    22: (0.0, 1.0),  # right_middle
    23: (0.0, 1.0),  # right_index
    24: (0.0, 1.0),  # right_thumb-bend
    25: (0.0, 1.0),  # right_thumb-rotation
    26: (0.0, 1.0),  # left_pinky
    27: (0.0, 1.0),  # left_ring
    28: (0.0, 1.0),  # left_middle
    29: (0.0, 1.0),  # left_index
    30: (0.0, 1.0),  # left_thumb-bend
    31: (0.0, 1.0),  # left_thumb-rotation
    32: (-0.5, 3.1),  # left_wrist
    33: (-4.5, -1.2),  # right_wrist
}
    # 32: (-0.65, 2.65),  # left_wrist
    # 33: (-4.0, -2.0),  # right_wrist