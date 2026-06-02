# uminoid_exo_interface
Manages values produced from an exoskeleton. Python acts as orchestrator; the C++ code under `/cpp` executes the values read from the exoskeleton.

## Prerequisites

```bash
# Build tools
sudo apt-get install git wget cmake build-essential

# yaml-cpp
sudo apt-get install libyaml-cpp-dev

# Librealsense dependencies
sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev

# Librealsense Linux backend
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
```

## One-time device setup

### Dynamixel arms — stable `/dev` symlinks

Run once per machine. Creates `/dev/left_arm` and `/dev/right_arm` matched by the FTDI chip's unique serial number, so the symlinks survive reboots and USB re-plugs regardless of port order.

Connect **one arm at a time** and find its FTDI serial:

```bash
ls -l /dev/serial/by-id/
```

Look for a line like:
```
usb-FTDI_USB__-__Serial_Converter_FT94EJO0-if00-port0 -> ../../ttyUSB0
```

The serial is the part between the last `_` and `-if00` — e.g. `FT94EJO0`. Then register it:

```bash
sudo $(which python) py/calibrate/setup_dynamixel_udev.py left_arm <serial>
```

Repeat for the right arm. Verify both:

```bash
ls -l /dev/left_arm /dev/right_arm
```

Expected output:
```
lrwxrwxrwx 1 root root 7 ... /dev/left_arm -> ttyUSB*
lrwxrwxrwx 1 root root 7 ... /dev/right_arm -> ttyUSB*
```

### Inspire hands — assign unique HAND\_IDs

Run once per hand. Plug in **one hand at a time**. The left hand keeps the factory default (ID=1); flash the right hand to ID=2.

```bash
python py/get_inspire_id.py ttyUSB0   # check current ID before flashing
python py/set_inspire_id.py ttyUSB0 --new-id 2
python py/get_inspire_id.py ttyUSB0   # verify
```

## Usage

### Python environment

```bash
conda create -n uminoid_exo python=3.11 && conda run -n uminoid_exo pip install -r requirements.txt
```

## Handle C++
```bash
cd cpp
```


### Configure CMAKE
(Only necessary when CMakeLists.txt is changed)
```bash
cmake -B build && cmake --build build
```


### Build
**Build all**
```bash
cmake --build build
```

**Build a specific file**
Build a specific File
```bash
cmake --build build --target <target_file>
```


### Execute
```bash
./build/<target_file>
```
