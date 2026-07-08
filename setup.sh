#!/bin/bash
# ═══════════════════════════════════════════════════════════
# Kinect for Linux — Multi-distro Installer
# Supports: Arch Linux, Ubuntu/Debian, Fedora
# License: MIT
# ═══════════════════════════════════════════════════════════

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

log()  { echo -e "${GREEN}[✓]${NC} $1"; }
warn() { echo -e "${YELLOW}[!]${NC} $1"; }
err()  { echo -e "${RED}[✗]${NC} $1"; }
info() { echo -e "${BLUE}[i]${NC} $1"; }

# ─── Distro Detection ─────────────────────────────
detect_distro() {
    if [ -f /etc/arch-release ]; then
        DISTRO="arch"
        PKG="pacman"
    elif [ -f /etc/debian_version ] || [ -f /etc/lsb-release ]; then
        DISTRO="debian"
        PKG="apt"
    elif [ -f /etc/fedora-release ]; then
        DISTRO="fedora"
        PKG="dnf"
    else
        DISTRO="unknown"
        PKG="unknown"
    fi
    info "Detected distro: $DISTRO"
}

# ─── Install Dependencies ─────────────────────────
install_deps() {
    info "Installing build dependencies..."
    case $DISTRO in
        arch)
            sudo pacman -Sy --noconfirm --needed \
                cmake gcc make pkg-config \
                libfreenect libusb alsa-lib pulseaudio qt6-base qt6-tools \
                python python-numpy opencv v4l2loopback-dkms
            ;;
        debian)
            sudo apt update
            sudo apt install -y \
                build-essential cmake pkg-config \
                libfreenect-dev libusb-1.0-0-dev libasound2-dev libpulse-dev \
                qt6-base-dev qt6-tools-dev libopencv-dev \
                python3 python3-numpy python3-opencv
            ;;
        fedora)
            sudo dnf install -y \
                cmake gcc make pkg-config \
                libusb1-devel alsa-lib-devel pulseaudio-libs-devel \
                qt6-qtbase-devel qt6-qtdeclarative-devel \
                python3 python3-numpy opencv-devel
            ;;
        *)
            err "Unsupported distro. Install deps manually:"
            echo "  cmake, gcc, make, libfreenect, libusb, alsa, pulseaudio, qt6-base, opencv"
            exit 1
            ;;
    esac
    log "Dependencies installed"
}

# ─── Firmware (Proprietary) ────────────────────────
install_firmware() {
    local FW_PATH="/usr/lib/firmware/kinect_uac_firmware.bin"
    if [ -f "$FW_PATH" ]; then
        log "Firmware already installed"
        return 0
    fi

    echo ""
    echo -e "${YELLOW}═══ Kinect UAC Firmware (Proprietary by Microsoft) ═══${NC}"
    echo ""
    echo "This firmware is required for motor/LED control."
    echo "It is NOT included for legal reasons."
    echo ""
    echo "Options:"
    echo "  1) I have the Kinect Runtime installer (.exe)"
    echo "  2) Download from Microsoft (opens browser)"
    echo "  3) Skip (depth + audio will work, motor/LED won't)"
    echo ""
    read -p "Choice [1/2/3]: " choice

    case $choice in
        1)
            read -p "Path to KinectRuntime-v1.x-Setup.exe: " exe_path
            if [ ! -f "$exe_path" ]; then
                err "File not found: $exe_path"
                return 1
            fi
            info "Extracting firmware..."
            tmpdir=$(mktemp -d)
            dd bs=1 skip=622624 count=115650048 if="$exe_path" of="$tmpdir/kr.cab" 2>/dev/null
            if command -v cabextract &>/dev/null; then
                cabextract -F a2 "$tmpdir/kr.cab" -d "$tmpdir/" 2>/dev/null
            else
                warn "cabextract not found. Installing..."
                case $DISTRO in
                    arch)    sudo pacman -S --noconfirm cabextract ;;
                    debian)  sudo apt install -y cabextract ;;
                    fedora)  sudo dnf install -y cabextract ;;
                esac
                cabextract -F a2 "$tmpdir/kr.cab" -d "$tmpdir/" 2>/dev/null
            fi
            if command -v msiextract &>/dev/null; then
                msiextract "$tmpdir/a2.msi" -d "$tmpdir/extracted/" 2>/dev/null
            else
                warn "msiextract not found. Installing cabextract..."
                case $DISTRO in
                    arch)    sudo pacman -S --noconfirm cabextract ;;
                    debian)  sudo apt install -y cabextract ;;
                    fedora)  sudo dnf install -y cabextract ;;
                esac
                msiextract "$tmpdir/a2.msi" -d "$tmpdir/extracted/" 2>/dev/null
            fi
            fw=$(find "$tmpdir" -name "UAC.bin" -o -name "audios.bin" 2>/dev/null | head -1)
            if [ -n "$fw" ]; then
                sudo cp "$fw" "$FW_PATH"
                log "Firmware installed to $FW_PATH"
            else
                err "Could not extract firmware. Try manual download."
            fi
            rm -rf "$tmpdir"
            ;;
        2)
            info "Opening Microsoft download page..."
            if command -v xdg-open &>/dev/null; then
                xdg-open "https://www.microsoft.com/en-us/download/details.aspx?id=40276"
            else
                echo "Download from: https://www.microsoft.com/en-us/download/details.aspx?id=40276"
                echo "Run this script again after downloading."
            fi
            exit 0
            ;;
        3)
            warn "Skipping firmware. Motor/LED won't work."
            ;;
    esac
}

# ─── Build ────────────────────────────────────────
build() {
    info "Building Kinect for Linux..."
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    cmake "$SCRIPT_DIR" \
        -DCMAKE_BUILD_TYPE=Release \
        -DOPENNI2_DIR=/home/rexx/Proyectos/OpenNI2
    make -j$(nproc)
    log "Build complete"
}

# ─── Install ──────────────────────────────────────
install_app() {
    info "Installing Kinect for Linux..."
    cd "$BUILD_DIR"
    sudo cmake --install .

    # udev rules
    sudo cp "$SCRIPT_DIR/90-kinect.rules" /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger

    # v4l2loopback config
    sudo cp "$SCRIPT_DIR/v4l2loopback.conf" /etc/modprobe.d/

    # Python venv for skeleton tracker
    info "Setting up Python environment for skeleton tracker..."
    python3 -m venv /tmp/mediapipe-venv 2>/dev/null || true
    /tmp/mediapipe-venv/bin/pip install mediapipe numpy opencv-python 2>/dev/null || true

    # Load v4l2loopback
    sudo modprobe v4l2loopback devices=1 video_nr=100 exclusive_caps=1 card_label="Kinect" 2>/dev/null || true

    log "Installation complete!"
    echo ""
    echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"
    echo -e "${GREEN} Kinect for Linux installed successfully!${NC}"
    echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"
    echo ""
    echo "  Run daemon:     sudo k4wd"
    echo "  Run GUI:        kinect-for-linux"
    echo "  OpenNI2 apps:   Set LD_LIBRARY_PATH=/usr/local/lib"
    echo ""
    echo "  Processing:     Copy libK4WDriver.so to SimpleOpenNI OpenNI2/Drivers/"
    echo ""
}

# ─── Main ─────────────────────────────────────────
main() {
    echo ""
    echo -e "${BLUE}═══ Kinect for Linux Installer ═══${NC}"
    echo ""

    detect_distro
    install_deps
    install_firmware
    build
    install_app
}

main "$@"
