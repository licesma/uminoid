# Run the Inspire hand from the host

Assumes the one-time setup in `inspire.md` is done (binaries built on the G1
and the host, correct hand ID hardcoded, `SerialPort::recv()` patched).

## 1. On the G1 — start the DDS ↔ serial bridge

```bash
ssh g1_robot                # password: 123 (or key-based if set up)
cd ~/uminoid_exo_interface/cpp/build
sudo ./dfx_inspire_service/inspire_g1
```

Expected output — then it just sits there waiting for DDS commands:

```
 --- Unitree Robotics ---
  Inspire Hand Controller
```

Leave this terminal open. The bridge needs `sudo` because it opens
`/dev/ttyUSB0`.

## 2. On the host — check the link to the G1 is up

```bash
ip -br link show | grep enx
```

The `enx…` interface to the G1 should say `UP`. If not, plug the USB-ethernet
cable in and re-check.

## 3. On the host — run the test client

```bash
cd ~/repos/uminoid_exo_interface/cpp/build
./inspire_g1_test enx00e04c6803fc
```

Pass the host's G1-facing interface name as the argument (the `enx…` one from
step 2). The program sweeps the right-hand **index finger** open ↔ closed
and prints `target` vs `actual`:

```
index target: 0.5  actual: 0.456
```

`actual` should track `target` with a small lag. Ctrl+C to stop — it returns
the hand to the open position before exiting.

## Signal flow (for reference)

```
host: inspire_g1_test  --DDS rt/inspire/cmd-->   G1: inspire_g1  --serial-->  Inspire hand
                       <--DDS rt/inspire/state--                <--serial--
```

If `actual` stays at 0 or the finger doesn't move, see `inspire.md`
("Two fixes required per-G1") — the bridge binary was probably built without
the correct hand ID or the `recv()` loop patch.
