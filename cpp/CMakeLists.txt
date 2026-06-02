cmake_minimum_required(VERSION 3.5)
project(uminoid_exo_interface VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
endif()


######################################################################################################
#                                         Submodules                                                 #
######################################################################################################

# Unitree SDK 
set(BUILD_EXAMPLES OFF CACHE BOOL "Skip building SDK examples" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/unitree_sdk2 unitree_sdk2)

# Librealsense
set(BUILD_EXAMPLES OFF CACHE BOOL "Skip building librealsense examples" FORCE)
set(BUILD_GRAPHICAL_EXAMPLES OFF CACHE BOOL "Skip building librealsense graphical examples" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/librealsense librealsense)

# DynamixelSDK
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/DynamixelSDK/c++ dynamixel_sdk)

# DFX Inspire Hand Service
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/dfx_inspire_service dfx_inspire_service)

# Pinocchio (rigid-body dynamics). Only RNEA + URDF parsing are used, so the
# Python interface, collision support, tests, benchmarks, and utils are all
# disabled to keep the build light. Requires liburdfdom-dev (apt) for URDF
# support. The build-tree RPATH lets executables find libpinocchio at runtime
# without any LD_LIBRARY_PATH setup.
set(BUILD_PYTHON_INTERFACE       OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_COLLISION_SUPPORT OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING                OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARK              OFF CACHE BOOL "" FORCE)
set(BUILD_UTILS                  OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_URDF_SUPPORT      ON  CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/pinocchio pinocchio)



######################################################################################################
#                                          Packages                                                  #
######################################################################################################

find_package(yaml-cpp REQUIRED)



######################################################################################################
#                                         Libraries                                                  #
######################################################################################################
# upper_body_reader owns the exoskeleton sensor reader implementations
add_library(
  upper_body_reader OBJECT
  upper_body_reader/exo_upper_body_reader.cpp
  upper_body_reader/arm_reader/arm_reader.cpp
  upper_body_reader/arm_reader/dynamixel/dynamixel_arm.cpp
  utils/dynamixel_timestamp_helper.cpp
)
target_include_directories(upper_body_reader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(upper_body_reader PUBLIC dynamixel_sdk yaml-cpp)
target_compile_definitions(upper_body_reader PRIVATE
  G1_BOUNDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/g1/model/upperBodyJointBounds.yaml"
  DYNAMIXEL_BOUNDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/upper_body_reader/arm_reader/dynamixel/dynamixel_bounds.yaml"
)

# in-process MJPEG preview server (shared by usb + zmq camera recorders)
find_library(JPEG_LIB jpeg REQUIRED)
add_library(preview_server OBJECT camera/preview_server.cpp)
target_include_directories(preview_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(preview_server PUBLIC ${JPEG_LIB} pthread)

# usb camera recorder (local RealSense via librealsense2)
add_library(usb_camera_recorder OBJECT camera/usb_camera_recorder.cpp)
target_include_directories(usb_camera_recorder PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(usb_camera_recorder PUBLIC realsense2 preview_server pthread)

# manus hand reader + zmq camera recorder (G1 head camera over ZMQ)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
add_library(manus_reader OBJECT
  manus/manus_reader.cpp
  manus/manus_glove_reader.cpp
  manus/manus_hand.cpp)
target_include_directories(manus_reader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${ZMQ_INCLUDE_DIRS})
target_link_libraries(manus_reader PUBLIC ${ZMQ_LIBRARIES})

add_library(zmq_camera_recorder OBJECT camera/zmq_camera_recorder.cpp)
target_include_directories(zmq_camera_recorder PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${ZMQ_INCLUDE_DIRS})
target_link_libraries(zmq_camera_recorder PUBLIC ${ZMQ_LIBRARIES} preview_server pthread)

# amo_bridge: ZMQ link to the Python AMO sidecar (state PUB / action SUB).
add_library(amo_bridge OBJECT g1/amo/amo_bridge.cpp g1/amo/amo_zmq_helper.cpp)
target_include_directories(amo_bridge PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${ZMQ_INCLUDE_DIRS})
target_link_libraries(amo_bridge PUBLIC ${ZMQ_LIBRARIES} unitree_sdk2 pthread)

# dynamics: per-mode joint dynamics (PD gains + feed-forward torque). Psi-0
# mode uses Pinocchio's RNEA on a reduced (arms-only) model loaded from the
# Psi-0 G1 URDF.
add_library(dynamics OBJECT g1/dynamics/dynamics.cpp)
target_include_directories(dynamics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(dynamics PUBLIC pinocchio::pinocchio)
target_compile_definitions(dynamics PRIVATE
  G1_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../third_party/Psi0/real/assets/g1/g1_body29_hand14.urdf")

# g1 robot + controller. Links against amo_bridge because G1Controller owns
# an AmoBridge member and calls publish_state / latest_action.
add_library(g1 OBJECT g1/g1Robot.cpp g1/g1Controller.cpp)
target_include_directories(g1 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(g1 PUBLIC unitree_sdk2 amo_bridge dynamics)
target_compile_definitions(g1 PRIVATE
  G1_BOUNDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/g1/model/upperBodyJointBounds.yaml"
  DYNAMIXEL_BOUNDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/upper_body_reader/arm_reader/dynamixel/dynamixel_bounds.yaml"
)

# bridge that drives g1Controller from arm-reader samples and records CSVs
add_library(g1_upper_body_reader OBJECT upper_body_reader/g1_upper_body_reader.cpp)
target_include_directories(g1_upper_body_reader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(g1_upper_body_reader PUBLIC g1 upper_body_reader)



######################################################################################################
#                                         Executables                                                  #
######################################################################################################


add_executable(collect collect.cpp)
target_include_directories(collect PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(collect PRIVATE upper_body_reader g1_upper_body_reader g1 amo_bridge dynamics hand_retargeter inspire_retargeter dex3_retargeter manus_reader usb_camera_recorder zmq_camera_recorder preview_server yaml-cpp)


file(GLOB TEST_SOURCES test/*.cpp)
# DDS-based tests that link against unitree_sdk2 instead of the default test deps
set(DDS_TESTS inspire_g1_test retarget_test camera_test inspire_force_test dex3_test)
foreach(test_source ${TEST_SOURCES})
  get_filename_component(test_name ${test_source} NAME_WE)
  if(${test_name} IN_LIST DDS_TESTS)
    continue()
  endif()
  add_executable(${test_name} ${test_source})
  target_include_directories(${test_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/dfx_inspire_service/include)
  target_link_libraries(${test_name} PRIVATE upper_body_reader manus_reader yaml-cpp)
  target_compile_definitions(${test_name} PRIVATE
    DYNAMIXEL_BOUNDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/upper_body_reader/arm_reader/dynamixel/dynamixel_bounds.yaml"
  )
endforeach()

add_executable(retarget_test test/retarget_test.cpp)
target_include_directories(retarget_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(retarget_test PRIVATE hand_retargeter inspire_retargeter manus_reader)

add_executable(camera_test test/camera_test.cpp)
target_include_directories(camera_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(camera_test PRIVATE usb_camera_recorder preview_server)


add_executable(collect_cli_test test/collect_cli/main.cpp)
target_include_directories(collect_cli_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# DDS-based inspire hand test (communicates via G1 DDS topics)
add_executable(inspire_g1_test test/inspire_g1_test.cpp)
target_link_libraries(inspire_g1_test PRIVATE unitree_sdk2)

# DDS-based dex3 hand test (subscribes to hand state, drives thumb_0 only)
add_executable(dex3_test test/dex3_test.cpp)
target_link_libraries(dex3_test PRIVATE unitree_sdk2 manus_reader yaml-cpp pthread)

# Live fingertip-force readout (USB or G1 DDS, selected via yaml config)
add_executable(inspire_force_test test/inspire_force_test.cpp)
target_include_directories(inspire_force_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/dfx_inspire_service/include)
target_link_libraries(inspire_force_test PRIVATE hand_retargeter inspire_retargeter manus_reader unitree_sdk2 yaml-cpp)

add_library(hand_retargeter OBJECT
  hand_retarget/hand_retargeter.cpp
)
target_include_directories(hand_retargeter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hand_retargeter PUBLIC manus_reader)

add_library(inspire_retargeter OBJECT
  hand_retarget/inspire/inspire_retargeter.cpp
  hand_retarget/inspire/usb_inspire_retargeter.cpp
  hand_retarget/inspire/g1_inspire_retargeter.cpp
  hand_retarget/inspire/inspire_port_resolver.cpp
)
target_include_directories(inspire_retargeter PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/dfx_inspire_service/include)
target_link_libraries(inspire_retargeter PUBLIC hand_retargeter yaml-cpp manus_reader unitree_sdk2)

add_library(dex3_retargeter OBJECT
  hand_retarget/dex3/single_dex3_controller.cpp
  hand_retarget/dex3/single_dex3_retargeter.cpp
  hand_retarget/dex3/dex3_retargeter.cpp
)
target_include_directories(dex3_retargeter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(dex3_retargeter PUBLIC hand_retargeter yaml-cpp manus_reader unitree_sdk2)



