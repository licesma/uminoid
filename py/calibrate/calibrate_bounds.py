import math
import sys
import time
import tty
import termios
import select

import yaml
from InquirerPy import inquirer

import joint_reader
from joints import ARM_JOINTS, JOINT_NAMES
from utils.convert_to_radian import convert_to_radian
from joint_reader_constants import ENCODER_PRECISION_RAD


ROBOT_BOUNDS_FILE = "./cpp/g1/model/upperBodyJointBounds.yaml"
JOINT_BOUNDS_FILE = "./cpp/upper_body_reader/arm_reader/as5600/as5600_bounds.yaml"


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
    jl = joint_bounds[full_name]["lower"]
    ju = joint_bounds[full_name]["upper"]
    delta = abs((robot_upper - robot_lower) - (ju - jl))
    if delta < 2 * ENCODER_PRECISION_RAD:
        return "[correct]"
    elif delta < 4 * ENCODER_PRECISION_RAD:
        return "[close]"
    else:
        return "[incorrect]"


def read_key_nonblocking():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def read_serial_value(csv_buffer, sock, joint_index):
    """Return the freshest encoder value for joint_index from relay via joint_reader."""
    try:
        csv_line, csv_buffer = joint_reader.read_freshest_csv_line(sock, csv_buffer)
        if csv_line is not None:
            parts = csv_line.split(",")
            if len(parts) == 15:           # t_us + 14 encoder values
                return float(parts[joint_index + 1]), csv_buffer
    except Exception:
        pass
    return None, csv_buffer


def calibrate_loop(sock, side, joint):
    full_name   = f"{side}_{joint}"
    joint_index = JOINT_NAMES.index(full_name)

    robot_bounds = load_yaml(ROBOT_BOUNDS_FILE)
    robot_lower  = robot_bounds[full_name]["lower"]
    robot_upper  = robot_bounds[full_name]["upper"]

    joint_bounds = load_yaml(JOINT_BOUNDS_FILE)

    joint_reader.drain_socket(sock)
    current_value = None
    csv_buffer = bytearray()

    while True:
        v, csv_buffer = read_serial_value(csv_buffer, sock, joint_index)
        if v is not None:
            current_value = v

        key = read_key_nonblocking()
        if key == "b":
            return
        elif key == "l" and current_value is not None:
            joint_bounds[full_name]["lower"] = convert_to_radian(current_value)
            save_yaml(JOINT_BOUNDS_FILE, joint_bounds)
        elif key == "u" and current_value is not None:
            joint_bounds[full_name]["upper"] = convert_to_radian(current_value)
            save_yaml(JOINT_BOUNDS_FILE, joint_bounds)

        jl = joint_bounds[full_name]["lower"]
        ju = joint_bounds[full_name]["upper"]

        robot_range = robot_upper - robot_lower
        joint_range = ju - jl if jl < ju else 2*math.pi + ju - jl
        delta_range = abs(robot_range - joint_range)

        if delta_range < 2 * ENCODER_PRECISION_RAD:
            color = "\033[32m"   # green
        elif delta_range < 4 * ENCODER_PRECISION_RAD:
            color = "\033[33m"   # yellow
        else:
            color = "\033[31m"   # red
        reset = "\033[0m"
        delta_tag = f"{color}[\u0394 Range {delta_range:.4f}]{reset}"

        output = [
            f"Side: {side.capitalize()},  Joint: {joint_display_name(joint)}  {delta_tag}\n",
            f"{'Robot Limits:':<16}  Lower: {robot_lower:<22.4f}  Upper: {robot_upper:<22.4f}  Range: {robot_range:.4f}",
            f"{'Joint Limits:':<16}  Lower: {jl:<22.4f}  Upper: {ju:<22.4f}  Range: {joint_range:.4f}\n",
        ]

        if current_value is not None:
            cv_rad = convert_to_radian(current_value)
            cv_str = f"{cv_rad:.4f} rad  (raw: {current_value:.0f})"
        else:
            cv_str = "—"

        output.append(f"Current value: {cv_str}\n")
        output.append("'b' -> go back   'l' -> set lower bound   'u' -> set upper bound")

        sys.stdout.write(f"\033[H{chr(10).join(output)}\033[J")
        sys.stdout.flush()

        time.sleep(0.05)


def main():
    sock = joint_reader.connect_to_relay()

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
                        "name": f"{j}  {joint_status_badge(side, j, robot_bounds, joint_bounds)}",
                        "value": j,
                    }
                    for j in ARM_JOINTS
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
                    calibrate_loop(sock, side, joint)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
