CODE_VERSION = "v2.1"
FRAME_RATE = 30
CHUNKS_SIZE = 1000
ROBOT_TYPE = "exoskeleton"
GAP_THRESHOLD = 100.0           # ms; flag episode if any host_timestamp gap exceeds this
DUPLICATION_THRESHOLD_RATIO = 0.10  # flag episode if fraction of consecutive-identical joint rows exceeds this

_LEFT_HAND_JOINTS = [
    "left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint", "left_hand_middle_1_joint",
    "left_hand_index_0_joint",  "left_hand_index_1_joint",
]
_RIGHT_HAND_JOINTS = [
    "right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
    "right_hand_index_0_joint",  "right_hand_index_1_joint",
    "right_hand_middle_0_joint", "right_hand_middle_1_joint",
]
_LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
]
_RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

STATE_SUFFIX = "_state"
ACTION_SUFFIX = "_action"

LEFT_HAND_STATE_COLS  = [c + STATE_SUFFIX  for c in _LEFT_HAND_JOINTS]
RIGHT_HAND_STATE_COLS = [c + STATE_SUFFIX  for c in _RIGHT_HAND_JOINTS]
LEFT_ARM_STATE_COLS   = [c + STATE_SUFFIX  for c in _LEFT_ARM_JOINTS]
RIGHT_ARM_STATE_COLS  = [c + STATE_SUFFIX  for c in _RIGHT_ARM_JOINTS]

LEFT_HAND_ACTION_COLS  = [c + ACTION_SUFFIX for c in _LEFT_HAND_JOINTS]
RIGHT_HAND_ACTION_COLS = [c + ACTION_SUFFIX for c in _RIGHT_HAND_JOINTS]
LEFT_ARM_ACTION_COLS   = [c + ACTION_SUFFIX for c in _LEFT_ARM_JOINTS]
RIGHT_ARM_ACTION_COLS  = [c + ACTION_SUFFIX for c in _RIGHT_ARM_JOINTS]

STATE_COLUMNS = [
    LEFT_HAND_STATE_COLS,
    RIGHT_HAND_STATE_COLS,
    LEFT_ARM_STATE_COLS,
    RIGHT_ARM_STATE_COLS,
]
ACTION_COLUMNS = [
    LEFT_HAND_ACTION_COLS,
    RIGHT_HAND_ACTION_COLS,
    LEFT_ARM_ACTION_COLS,
    RIGHT_ARM_ACTION_COLS,
]

HAND_SLOTS = 7           # G1 Dex3 per-hand DOFs
ARM_SLOTS = 7
TORSO_SLOTS = 4
BASE_SLOTS = 4
PSI0_DIM = 2 * HAND_SLOTS + 2 * ARM_SLOTS + TORSO_SLOTS + BASE_SLOTS  # 36
