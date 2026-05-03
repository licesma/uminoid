import serial
import struct
import sys

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 230400

FRAME_HEADER = 0xAA55
FRAME_FMT = "<HQ14H"
FRAME_PAYLOAD_SIZE = struct.calcsize(FRAME_FMT)  # 38
FRAME_SIZE = FRAME_PAYLOAD_SIZE + 2               # +CRC = 40


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def binary_to_csv(payload: bytes) -> str:
    header, ts, *vals = struct.unpack(FRAME_FMT, payload)
    return ",".join([str(ts)] + [str(v) for v in vals])


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else SERIAL_PORT
    ser = serial.Serial(port, SERIAL_BAUD, timeout=1)
    print(f"Reading {port} @ {SERIAL_BAUD}", file=sys.stderr)

    buf = bytearray()

    while True:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue

        buf += chunk

        while len(buf) >= FRAME_SIZE:
            sync = buf.find(b'\x55\xAA')
            if sync < 0:
                buf.clear()
                break
            if sync > 0:
                del buf[:sync]
            if len(buf) < FRAME_SIZE:
                break

            frame = bytes(buf[:FRAME_SIZE])
            del buf[:FRAME_SIZE]

            payload = frame[:FRAME_PAYLOAD_SIZE]
            crc_recv = struct.unpack("<H", frame[-2:])[0]
            crc_calc = crc16_ccitt(payload[2:])

            if crc_calc != crc_recv:
                print("[!] CRC mismatch, skipping frame", file=sys.stderr)
                continue

            print(binary_to_csv(payload))


if __name__ == "__main__":
    main()
