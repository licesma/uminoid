import socket
import serial
import threading

# ---- constants ----
SERIAL_PORT = "/dev/ttyUSB0"   # change to your Linux port (or /dev/serial/by-id/...)
SERIAL_BAUD = 230400

FRAME_SIZE = 40               # 2+8+28+2

TCP_HOST = "127.0.0.1"
TCP_PORT = 5000
# -------------------

clients = set()
clients_lock = threading.Lock()


def accept_loop(server_sock: socket.socket):
  while True:
    conn, addr = server_sock.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    with clients_lock:
      clients.add(conn)
    print(f"[+] client connected: {addr}")


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


def main():
  # TCP server
  server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  server.bind((TCP_HOST, TCP_PORT))
  server.listen(16)
  print(f"Listening on {TCP_HOST}:{TCP_PORT}")

  t = threading.Thread(target=accept_loop, args=(server,), daemon=True)
  t.start()

  # Serial
  ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
  print(f"Reading serial {SERIAL_PORT} @ {SERIAL_BAUD}")

  buf = bytearray()

  while True:
    chunk = ser.read(ser.in_waiting or 1)
    if not chunk:
      continue

    buf += chunk

    while len(buf) >= FRAME_SIZE:
      # Sync: find the 0xAA55 header (little-endian: 0x55 then 0xAA)
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
      broadcast(frame)


if __name__ == "__main__":
  main()