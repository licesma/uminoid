import math
"""
################################################################################
#  ⚠️  KEEP IN SYNC  ⚠️
#  This file MUST be kept in sync with: cpp/joint_reader/constants.hpp
################################################################################
"""
ENCODER_RESOLUTION = 4096
ENCODER_PRECISION = 1 / ENCODER_RESOLUTION
ENCODER_PRECISION_RAD = ENCODER_PRECISION * 2 * math.pi
