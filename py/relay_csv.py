import socket
import struct
import sys
import threading

TCP_HOST = "127.0.0.1"
TCP_PORT = 5000

FRAME_HEADER = 0xAA55
FRAME_FMT = "<HQ14H"
FRAME_PAYLOAD_SIZE = struct.calcsize(FRAME_FMT)  # 38

clients = set()
clients_lock = threading.Lock()


def accept_loop(server_sock: socket.socket):
    while True:
        conn, addr = server_sock.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with clients_lock:
            clients.add(conn)
        print(f"[+] client connected: {addr}", file=sys.stderr)


def broadcast(data: bytes):
    dead = []
    with clients_lock:
        for c in clients:
            try:
                c.sendall(data)
            except Exception:
                dead.append(c)
        for c in dead:
            try:
                c.close()
            except Exception:
                pass
            clients.discard(c)


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def csv_to_frame(csv_line: str) -> bytes:
    parts = csv_line.strip().split(",")
    ts = int(parts[0])
    vals = [int(v) for v in parts[1:16]]
    while len(vals) < 14:
        vals.append(0)

    payload = struct.pack(FRAME_FMT, FRAME_HEADER, ts, *vals[:14])
    crc = crc16_ccitt(payload[2:])
    return payload + struct.pack("<H", crc)


def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((TCP_HOST, TCP_PORT))
    server.listen(16)
    print(f"Listening on {TCP_HOST}:{TCP_PORT}", file=sys.stderr)

    t = threading.Thread(target=accept_loop, args=(server,), daemon=True)
    t.start()

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            frame = csv_to_frame(line)
            broadcast(frame)
        except (ValueError, struct.error) as e:
            print(f"[!] bad line: {e}", file=sys.stderr)


if __name__ == "__main__":
    main()
