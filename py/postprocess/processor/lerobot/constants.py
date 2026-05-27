CODE_VERSION = "v2.1"
FRAME_RATE = 30
CHUNKS_SIZE = 1000
ROBOT_TYPE = "exoskeleton"
GAP_THRESHOLD = 100.0           # ms; flag episode if any host_timestamp gap exceeds this
DUPLICATION_THRESHOLD_RATIO = 0.10  # flag episode if fraction of consecutive-identical joint rows exceeds this

LEFT_HAND_COLS = [
    "left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint", "left_hand_middle_1_joint",
    "left_hand_index_0_joint",  "left_hand_index_1_joint",
]
RIGHT_HAND_COLS = [
    "right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
    "right_hand_index_0_joint",  "right_hand_index_1_joint",
    "right_hand_middle_0_joint", "right_hand_middle_1_joint",
]
LEFT_ARM_COLS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
]
RIGHT_ARM_COLS = [
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

HAND_SLOTS = 7           # G1 Dex3 per-hand DOFs
ARM_SLOTS = 7
TORSO_SLOTS = 4
BASE_SLOTS = 4
PSI0_DIM = 2 * HAND_SLOTS + 2 * ARM_SLOTS + TORSO_SLOTS + BASE_SLOTS  # 36
