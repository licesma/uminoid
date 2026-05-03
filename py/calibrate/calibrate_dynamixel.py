#!/usr/bin/env python3

import argparse
import math
import select
import sys
import termios
import time
import tty

import yaml
from InquirerPy import inquirer

from joint_reader_constants import ENCODER_PRECISION_RAD
from joints import ARM_JOINTS
from read_dynamixel import BAUDRATE, DXL_IDS, UNAVAILABLE_VALUE, DynamixelPortReader, read_with_fallback
from utils.convert_to_radian import convert_to_radian


ROBOT_BOUNDS_FILE = "./cpp/g1/model/upperBodyJointBounds.yaml"
JOINT_BOUNDS_FILE = "./cpp/upper_body_reader/arm_reader/dynamixel/dynamixel_bounds.yaml"


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def save_yaml(path, data):
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False)


def joint_display_name(name):
    return " ".join(word.capitalize() for word in name.split("_"))


def joint_status_badge(side, joint, robot_bounds, joint_bounds):
    full_name = f"{side}_{joint}"
    robot_lower = robot_bounds[full_name]["lower"]
    robot_upper = robot_bounds[full_name]["upper"]
    joint_lower = joint_bounds[full_name]["lower"]
    joint_upper = joint_bounds[full_name]["upper"]
    delta = abs((robot_upper - robot_lower) - (joint_upper - joint_lower))
    if delta < 2 * ENCODER_PRECISION_RAD:
        return "[correct]"
    if delta < 4 * ENCODER_PRECISION_RAD:
        return "[close]"
    return "[incorrect]"


def read_key_nonblocking():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def read_joint_value(reader, joint):
    _, values, error = read_with_fallback(reader)
    if error:
        return None, error

    dxl_id = DXL_IDS[ARM_JOINTS.index(joint)]
    value = values.get(dxl_id)
    if value == UNAVAILABLE_VALUE:
        return None, f"{reader.name}: unavailable, using {UNAVAILABLE_VALUE}"
    return value, None


def calibrate_loop(reader, side, joint):
    full_name = f"{side}_{joint}"

    robot_bounds = load_yaml(ROBOT_BOUNDS_FILE)
    robot_lower = robot_bounds[full_name]["lower"]
    robot_upper = robot_bounds[full_name]["upper"]

    current_value = None
    last_error = None

    while True:
        value, error = read_joint_value(reader, joint)
        if value is not None:
            current_value = value
        last_error = error

        key = read_key_nonblocking()
        if key == "b":
            return
        elif key == "l" and current_value is not None:
            joint_bounds = load_yaml(JOINT_BOUNDS_FILE)
            joint_bounds[full_name]["lower"] = convert_to_radian(current_value)
            save_yaml(JOINT_BOUNDS_FILE, joint_bounds)
        elif key == "u" and current_value is not None:
            joint_bounds = load_yaml(JOINT_BOUNDS_FILE)
            joint_bounds[full_name]["upper"] = convert_to_radian(current_value)
            save_yaml(JOINT_BOUNDS_FILE, joint_bounds)

        joint_bounds = load_yaml(JOINT_BOUNDS_FILE)
        joint_lower = joint_bounds[full_name]["lower"]
        joint_upper = joint_bounds[full_name]["upper"]

        robot_range = robot_upper - robot_lower
        joint_range = joint_upper - joint_lower if joint_lower < joint_upper else 2 * math.pi + joint_upper - joint_lower
        delta_range = abs(robot_range - joint_range)

        if delta_range < 2 * ENCODER_PRECISION_RAD:
            color = "\033[32m"
        elif delta_range < 4 * ENCODER_PRECISION_RAD:
            color = "\033[33m"
        else:
            color = "\033[31m"
        reset = "\033[0m"
        delta_tag = f"{color}[Δ Range {delta_range:.4f}]{reset}"

        output = [
            f"Side: {side.capitalize()},  Joint: {joint_display_name(joint)}  {delta_tag}\n",
            f"{'Robot Limits:':<16}  Lower: {robot_lower:<22.4f}  Upper: {robot_upper:<22.4f}  Range: {robot_range:.4f}",
            f"{'Joint Limits:':<16}  Lower: {joint_lower:<22.4f}  Upper: {joint_upper:<22.4f}  Range: {joint_range:.4f}\n",
        ]

        if current_value is not None:
            current_value_rad = convert_to_radian(current_value)
            current_value_str = f"{current_value_rad:.4f} rad  (raw: {current_value})"
        else:
            current_value_str = "unavailable"

        output.append(f"Current value: {current_value_str}")
        output.append(f"Reader port: {reader.device}")
        if last_error:
            output.append(f"Status: {last_error}")
        output.append("")
        output.append("'b' -> go back   'l' -> set lower bound   'u' -> set upper bound")

        sys.stdout.write(f"\033[H{chr(10).join(output)}\033[J")
        sys.stdout.flush()

        time.sleep(0.05)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Interactively calibrate Dynamixel joint bounds."
    )
    parser.add_argument("--left-port", default="/dev/left_arm", help="Left arm serial port")
    parser.add_argument("--right-port", default="/dev/right_arm", help="Right arm serial port")
    parser.add_argument("--baudrate", type=int, default=BAUDRATE, help="Dynamixel bus baudrate")
    return parser.parse_args()


def main():
    args = parse_args()
    readers = {
        "left": DynamixelPortReader("left", args.left_port, args.baudrate),
        "right": DynamixelPortReader("right", args.right_port, args.baudrate),
    }

    for reader in readers.values():
        try:
            reader.open()
        except Exception:
            reader.close()

    try:
        while True:
            sys.stdout.write("\033[H\033[J")
            sys.stdout.flush()
            side = inquirer.select(
                message="Select side:",
                choices=["left", "right"],
            ).execute()

            while True:
                sys.stdout.write(f"\033[H\033[JSide: {side.capitalize()}\n\n")
                sys.stdout.flush()

                robot_bounds = load_yaml(ROBOT_BOUNDS_FILE)
                joint_bounds = load_yaml(JOINT_BOUNDS_FILE)
                joint_choices = [
                    {
                        "name": f"{joint}  {joint_status_badge(side, joint, robot_bounds, joint_bounds)}",
                        "value": joint,
                    }
                    for joint in ARM_JOINTS
                ] + [{"name": "← back", "value": "← back"}]

                joint = inquirer.select(
                    message="Select joint:",
                    choices=joint_choices,
                ).execute()

                if joint == "← back":
                    break

                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                    tty.setcbreak(fd)
                    calibrate_loop(readers[side], side, joint)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        for reader in readers.values():
            reader.close()


if __name__ == "__main__":
    main()
