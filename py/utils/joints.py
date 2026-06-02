ARM_JOINTS = [
    "shoulder_pitch",
    "shoulder_roll",
    "shoulder_yaw",
    "elbow",
    "wrist_roll",
    "wrist_pitch",
    "wrist_yaw",
]

JOINT_NAMES = [f"left_{n}" for n in ARM_JOINTS] + [f"right_{n}" for n in ARM_JOINTS]

def joint_name(index: int) -> str:
    return JOINT_NAMES[index]
