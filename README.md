# Kinect for Linux

Kinect v1 (Xbox 360 / Kinect for Windows) support for Linux.

**License:** MIT

## Features

- **Daemon (k4wd)** — Owns the real Kinect USB device, serves depth/video/audio to other apps
- **GUI (kinect-for-linux)** — Control panel with camera view, motor control, audio, skeleton tracking
- **OpenNI2 Driver** — Makes Kinect recognizable by Processing, NiTE, and other OpenNI2 apps
- **Skeleton Tracking** — MediaPipe Pose via Python, integrated into Supercam mode

## Quick Install

```bash
git clone <repo>
cd kinect-for-linux
./setup.sh
```

## Manual Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo cmake --install .
```

## Dependencies

### Arch Linux
```bash
sudo pacman -S cmake gcc make libfreenect libusb alsa-lib pulseaudio qt6-base python opencv
```

### Ubuntu/Debian
```bash
sudo apt install build-essential cmake libfreenect-dev libusb-1.0-0-dev libasound2-dev libpulse-dev qt6-base-dev python3 opencv
```

### Fedora
```bash
sudo dnf install cmake gcc make libusb1-devel alsa-lib-devel pulseaudio-libs-devel qt6-qtbase-devel python3 opencv-devel
```

## Usage

```bash
# Start daemon (needs root for USB access)
sudo k4wd

# Start GUI (in another terminal)
kinect-for-linux
```

## Kinect Firmware

The Kinect for Windows (K4W) model requires proprietary Microsoft UAC firmware for motor/LED control. The installer will guide you through obtaining it.

Without firmware: depth + audio work normally. Motor + LED won't work.

## OpenNI2 Integration

For Processing/SimpleOpenNI:
```bash
# Copy driver to SimpleOpenNI's OpenNI2/Drivers/
cp /usr/local/lib/OpenNI2/Drivers/libK4WDriver.so \
   ~/sketchbook/libraries/SimpleOpenNI/library/linux64/OpenNI2/Drivers/

# Set library path
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

## Architecture

```
k4wd daemon (root, owns USB)
    ├── SHM: depth (640x480 uint16)
    ├── SHM: video (640x480 RGB)
    ├── V4L2 loopback (/dev/video100)
    ├── PulseAudio null-sink + remap source
    └── Unix socket (/tmp/k4w.sock)
         │
         ├── kinect-for-linux (GUI, reads SHM)
         ├── OpenNI2 apps (via libK4WDriver.so, reads SHM)
         └── Discord/Zoom (via PulseAudio + V4L2)
```

## Supported Hardware

- Kinect for Windows v1 (K4W 1473) — Full support
- Xbox 360 Kinect — Depth + audio (motor needs different firmware)
