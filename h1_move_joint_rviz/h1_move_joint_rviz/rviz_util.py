LIMITS_URDF = {
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
    20: (1.7, 0.0),  # right_pinky
    21: (1.7, 0.0),  # right_ring
    22: (1.7, 0.0),  # right_middle
    23: (1.7, 0.0),  # right_index
    24: (0.6, 0.0),  # right_thumb_bend
    25: (1.3, 0.0),  # right_thumb_rotation
    26: (1.7, 0.0),  # left_pinky
    27: (1.7, 0.0),  # left_ring
    28: (1.7, 0.0),  # left_middle
    29: (1.7, 0.0),  # left_index
    30: (0.6, 0.0),  # left_thumb_bend
    31: (1.3, 0.0),  # left_thumb_rotation
    32: (-3.05, 3.05),  # left_wrist
    33: (-3.05, 3.05),  # right_wrist
}

START_POSITION = {
    0: 0.0,  # right_hip_roll_joint
    1: 0.0,  # right_hip_pitch_joint
    2: 0.0,  # right_knee_joint
    3: 0.0,  # left_hip_roll_joint
    4: 0.0,  # left_hip_pitch_joint
    5: 0.0,  # left_knee_joint
    6: 0.0,  # torso_joint
    7: 0.0,  # left_hip_yaw_joint
    8: 0.0,  # right_hip_yaw_joint
    9: 0.0,  # IMPACT
    10: 0.0,  # left_ankle_joint
    11: 0.0,  # right_ankle_joint
    12: 0.0,  # right_shoulder_pitch_joint
    13: 0.0,  # right_shoulder_roll_joint
    14: 0.0,  # right_shoulder_yaw_joint
    15: 0.0,  # right_elbow_joint
    16: 0.0,  # left_shoulder_pitch_joint
    17: 0.0,  # left_shoulder_roll_joint
    18: 0.0,  # left_shoulder_yaw_joint
    19: 0.0,  # left_elbow_joint
    20: 1.0,  # right_pinky
    21: 1.0,  # right_ring
    22: 1.0,  # right_middle
    23: 1.0,  # right_index
    24: 1.0,  # right_thumb_bend
    25: 1.0,  # right_thumb_rotation
    26: 1.0,  # left_pinky
    27: 1.0,  # left_ring
    28: 1.0,  # left_middle
    29: 1.0,  # left_index
    30: 1.0,  # left_thumb-bend
    31: 1.0,  # left_thumb-rotation
    32: 1.65,  # left_wrist
    33: -3.03,  # right_wrist
}

JOINTS_NAMES = [
    "right_hip_roll_joint", 
    "right_hip_pitch_joint", 
    "right_knee_joint",
    "left_hip_roll_joint", 
    "left_hip_pitch_joint", 
    "left_knee_joint",
    "torso_joint", 
    "left_hip_yaw_joint", 
    "right_hip_yaw_joint",
    "left_ankle_joint",
    "right_ankle_joint",
    "right_shoulder_pitch_joint",  
    "right_shoulder_roll_joint", 
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint", 
    "left_shoulder_yaw_joint", 
    "left_elbow_joint",
    "R_pinky_proximal_joint",
    "R_ring_proximal_joint",
    "R_middle_proximal_joint",
    "R_index_proximal_joint",
    "R_thumb_proximal_pitch_joint",
    "R_thumb_proximal_yaw_joint",
    "L_pinky_proximal_joint",
    "L_ring_proximal_joint",
    "L_middle_proximal_joint",
    "L_index_proximal_joint",
    "L_thumb_proximal_pitch_joint",
    "L_thumb_proximal_yaw_joint",
    "left_hand_joint",
    "right_hand_joint",
]

def map_range(
    value: float,
    in_min: float,
    in_max: float,
    out_min: float,
    out_max: float,
) -> float:
    """
    Linearly map a value from one numerical range to another.

    This function performs a linear transformation of the input value from
    the original range [in_min, in_max] to the target range [out_min, out_max]
    The formula used is:
        output = (value - in_min) * (out_max - out_min) /
            (in_max - in_min) + out_min

    Args:
        value: Input value to be mapped
        in_min: Minimum value of the original range
        in_max: Maximum value of the original range
        out_min: Minimum value of the target range
        out_max: Maximum value of the target range

    Returns:
        float: Value mapped to the target range

    Example:
        >>> map_range(5, 0, 10, 0, 100)
        50.0
    """
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min