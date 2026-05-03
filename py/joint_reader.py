import socket
import struct

# Relay connection (relay.py must be running)
TCP_HOST = "127.0.0.1"
TCP_PORT = 5000

FRAME_HEADER = 0xAA55
FRAME_FMT = "<HQ14H"
FRAME_PAYLOAD_SIZE = struct.calcsize(FRAME_FMT)   # 38
FRAME_SIZE = FRAME_PAYLOAD_SIZE + 2                # +CRC


def connect_to_relay(host: str = TCP_HOST, port: int = TCP_PORT, timeout: float = 0.1) -> socket.socket:
    """Connect to the relay TCP server. Ensure relay.py is running first."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.connect((host, port))
    sock.settimeout(timeout)
    return sock


def drain_socket(sock: socket.socket):
    """Discard all pending data from the socket (equivalent to serial reset_input_buffer)."""
    sock.settimeout(0)
    try:
        while True:
            data = sock.recv(4096)
            if not data:
                break
    except (BlockingIOError, socket.timeout):
        pass
    finally:
        sock.settimeout(0.1)


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def binary_to_csv(payload: bytes) -> str:
    """Transform binary payload to CSV line (ts, v0, v1, ..., v13)."""
    header, ts, *vals = struct.unpack(FRAME_FMT, payload)
    return ",".join([str(ts)] + [str(v) for v in vals])


def read_one_csv_line(sock: socket.socket, buffer: bytearray, timeout: float = 0.1):
    """
    Try to read one CSV line from relay TCP stream. Returns (csv_line or None, updated_buffer).
    Use a short socket timeout for non-blocking behavior.
    """
    if len(buffer) < FRAME_SIZE:
        sock.settimeout(timeout)
        try:
            chunk = sock.recv(4096)
            if chunk:
                buffer += chunk
        except socket.timeout:
            pass

    while len(buffer) >= FRAME_SIZE:
        if buffer[0:2] != b'\x55\xAA':
            buffer.pop(0)
            continue

        frame = bytes(buffer[:FRAME_SIZE])
        del buffer[:FRAME_SIZE]

        payload = frame[:FRAME_PAYLOAD_SIZE]
        crc_recv = struct.unpack("<H", frame[-2:])[0]
        crc_calc = crc16_ccitt(payload[2:])

        if crc_calc != crc_recv:
            continue

        return binary_to_csv(payload), buffer

    return None, buffer


def read_freshest_csv_line(sock: socket.socket, buffer: bytearray, timeout: float = 0.1):
    """
    Drain all available frames and return the freshest CSV line.
    Use when you want the most recent reading, not the oldest in the buffer.
    """
    last_csv = None
    while True:
        csv_line, buffer = read_one_csv_line(sock, buffer, timeout)
        if csv_line is None:
            return last_csv, buffer
        last_csv = csv_line


def iter_csv_lines(sock: socket.socket, timeout: float = 0.1):
    """Yield CSV lines from relay TCP stream."""
    buffer = bytearray()
    while True:
        csv_line, buffer = read_one_csv_line(sock, buffer, timeout)
        while csv_line is not None:
            yield csv_line
            csv_line, buffer = read_one_csv_line(sock, buffer, timeout)
        sock.settimeout(timeout)
        try:
            chunk = sock.recv(4096)
            if chunk:
                buffer += chunk
        except socket.timeout:
            pass


if __name__ == "__main__":
    sock = connect_to_relay()
    try:
        for csv_line in iter_csv_lines(sock):
            print(csv_line)
    finally:
        sock.close()
