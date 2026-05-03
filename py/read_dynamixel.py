#!/usr/bin/env python3

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from joints import ARM_JOINTS

try:
    from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler
except ImportError:
    sdk_path = Path(__file__).resolve().parent / "third_party" / "DynamixelSDK" / "python" / "src"
    sys.path.insert(0, str(sdk_path))
    from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler


PROTOCOL_VERSION = 2.0
BAUDRATE = 1_000_000
DXL_IDS = tuple(range(0, 7))
DXL_POSITION_MODULUS = 4096
UNAVAILABLE_VALUE = 5000

ADDR_REALTIME_TICK = 120
LEN_REALTIME_TICK = 2
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
SYNC_READ_START = ADDR_REALTIME_TICK
SYNC_READ_LEN = (ADDR_PRESENT_POSITION + LEN_PRESENT_POSITION) - ADDR_REALTIME_TICK
TICK_MAX = 32768


@dataclass
class TimestampUnwrapper:
    last_tick: int | None = None
    accumulated_ms: int = 0

    def unwrap(self, tick: int) -> int:
        tick &= 0x7FFF
        if self.last_tick is None:
            self.last_tick = tick
            return self.accumulated_ms

        delta = tick - self.last_tick
        if delta < 0:
            delta += TICK_MAX

        self.accumulated_ms += delta
        self.last_tick = tick
        return self.accumulated_ms


class DynamixelPortReader:
    def __init__(self, name: str, device: str, baudrate: int) -> None:
        self.name = name
        self.device = device
        self.baudrate = baudrate
        self.port_handler = PortHandler(device)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            SYNC_READ_START,
            SYNC_READ_LEN,
        )
        self.timestamp_unwrapper = TimestampUnwrapper()
        self.is_open = False
        self.available = False

    def open(self) -> None:
        if not self.port_handler.openPort():
            raise RuntimeError(f"{self.name}: failed to open port {self.device}")
        self.is_open = True

        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f"{self.name}: failed to set baudrate {self.baudrate}")

        for dxl_id in DXL_IDS:
            if not self.group_sync_read.addParam(dxl_id):
                raise RuntimeError(f"{self.name}: addParam failed for ID {dxl_id}")

        self.available = True

    def close(self) -> None:
        try:
            self.group_sync_read.clearParam()
        except Exception:
            pass

        if self.is_open:
            self.port_handler.closePort()
            self.is_open = False

        self.available = False

    def read(self) -> tuple[int, dict[int, int]]:
        result = self.group_sync_read.fastSyncRead()
        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"{self.name}: {self.packet_handler.getTxRxResult(result)}"
            )

        anchor_id = DXL_IDS[0]
        if not self.group_sync_read.isAvailable(anchor_id, ADDR_REALTIME_TICK, LEN_REALTIME_TICK):
            raise RuntimeError(f"{self.name}: realtime tick not available for ID {anchor_id}")

        raw_tick = self.group_sync_read.getData(anchor_id, ADDR_REALTIME_TICK, LEN_REALTIME_TICK)
        timestamp_ms = self.timestamp_unwrapper.unwrap(raw_tick)

        values: dict[int, int] = {}
        for dxl_id in DXL_IDS:
            if not self.group_sync_read.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                raise RuntimeError(f"{self.name}: present position not available for ID {dxl_id}")

            pos = self.group_sync_read.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            values[dxl_id] = pos % DXL_POSITION_MODULUS

        return timestamp_ms, values


def unavailable_values() -> dict[int, int]:
    return {dxl_id: UNAVAILABLE_VALUE for dxl_id in DXL_IDS}


def read_with_fallback(
    reader: DynamixelPortReader,
) -> tuple[int | None, dict[int, int], str | None]:
    if not reader.available:
        return None, unavailable_values(), f"{reader.name}: unavailable, using {UNAVAILABLE_VALUE}"

    try:
        timestamp_ms, values = reader.read()
        return timestamp_ms, values, None
    except Exception as exc:
        reader.close()
        return None, unavailable_values(), f"{exc}; using {UNAVAILABLE_VALUE}"


def clear_screen() -> None:
    sys.stdout.write("\033[H\033[J")


def render_output(
    left_timestamp_ms: int | None,
    right_timestamp_ms: int | None,
    left_values: dict[int, int] | None,
    right_values: dict[int, int] | None,
    left_port: str,
    right_port: str,
    baudrate: int,
    errors: list[str],
) -> str:
    lines = [
        "Dynamixel live reader",
        "",
        f"Left  port timestamp : {left_timestamp_ms if left_timestamp_ms is not None else '--'} ms",
        f"Right port timestamp : {right_timestamp_ms if right_timestamp_ms is not None else '--'} ms",
        "",
        f"{'Joint':<18}{'Left(ID)':>12}{'Right(ID)':>14}",
        "-" * 44,
    ]

    for joint_index, joint_name in enumerate(ARM_JOINTS):
        dxl_id = DXL_IDS[joint_index]
        left_value = "--" if left_values is None else str(left_values.get(dxl_id, "--"))
        right_value = "--" if right_values is None else str(right_values.get(dxl_id, "--"))
        lines.append(f"{joint_name:<18}{left_value:>12}{right_value:>14}")

    lines.extend(
        [
            "",
            f"Ports: left={left_port} right={right_port} baud={baudrate}",
            "Press Ctrl-C to stop.",
        ]
    )

    if errors:
        lines.append("")
        lines.extend(f"Status: {error}" for error in errors)

    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Continuously read 7 Dynamixels from two USB serial ports."
    )
    parser.add_argument("--left-port", default="/dev/left_arm", help="Left arm serial port")
    parser.add_argument("--right-port", default="/dev/right_arm", help="Right arm serial port")
    parser.add_argument("--baudrate", type=int, default=BAUDRATE, help="Dynamixel bus baudrate")
    parser.add_argument(
        "--period",
        type=float,
        default=0.05,
        help="Polling period in seconds",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    readers = [
        DynamixelPortReader("left", args.left_port, args.baudrate),
        DynamixelPortReader("right", args.right_port, args.baudrate),
    ]

    try:
        for reader in readers:
            try:
                reader.open()
            except Exception:
                reader.close()

        left_timestamp_ms = None
        right_timestamp_ms = None
        left_values = unavailable_values()
        right_values = unavailable_values()

        while True:
            errors: list[str] = []
            left_timestamp_ms, left_values, left_error = read_with_fallback(readers[0])
            right_timestamp_ms, right_values, right_error = read_with_fallback(readers[1])
            if left_error:
                errors.append(left_error)
            if right_error:
                errors.append(right_error)

            clear_screen()
            sys.stdout.write(
                render_output(
                    left_timestamp_ms,
                    right_timestamp_ms,
                    left_values,
                    right_values,
                    args.left_port,
                    args.right_port,
                    args.baudrate,
                    errors,
                )
            )
            sys.stdout.write("\n")
            sys.stdout.flush()
            time.sleep(args.period)

    except KeyboardInterrupt:
        sys.stdout.write("\nStopping Dynamixel reader.\n")
        return 0
    finally:
        for reader in readers:
            reader.close()


if __name__ == "__main__":
    raise SystemExit(main())
