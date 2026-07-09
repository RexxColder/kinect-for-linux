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
        # Detect AUR helper (yay > paru > pacman only)
        if command -v yay &>/dev/null; then
            AUR_HELPER="yay"
        elif command -v paru &>/dev/null; then
            AUR_HELPER="paru"
        else
            AUR_HELPER=""
            warn "No AUR helper found (yay/paru). AUR packages may fail."
        fi
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

# ─── Arch: install AUR package ────────────────────
# Tries: yay -S / paru -S / pacman -S
aur_install() {
    local pkgs=("$@")
    if [ -n "$AUR_HELPER" ]; then
        info "Installing AUR packages with $AUR_HELPER: ${pkgs[*]}"
        $AUR_HELPER -S --noconfirm --needed "${pkgs[@]}"
    else
        info "Falling back to pacman for: ${pkgs[*]}"
        sudo pacman -S --noconfirm --needed "${pkgs[@]}"
    fi
}

# ─── Install Dependencies ─────────────────────────
install_deps() {
    info "Installing build dependencies..."
    case $DISTRO in
        arch)
            # Official repos
            sudo pacman -Sy --noconfirm --needed \
                cmake gcc make pkg-config \
                libusb alsa-lib qt6-base qt6-tools \
                python python-numpy opencv msitools \
                pipewire libjpeg-turbo
            # AUR packages (libfreenect, v4l2loopback-dkms, openni2)
            # pulseaudio-libs is NOT needed when pipewire-pulse is installed
            local AUR_PKGS=()
            pacman -Qi libfreenect &>/dev/null || AUR_PKGS+=(libfreenect)
            pacman -Qi v4l2loopback-dkms &>/dev/null || AUR_PKGS+=(v4l2loopback-dkms)
            pacman -Qi openni2 &>/dev/null || AUR_PKGS+=(openni2)
            pacman -Qi arrpc &>/dev/null || AUR_PKGS+=(arrpc)
            if [ ${#AUR_PKGS[@]} -gt 0 ]; then
                aur_install "${AUR_PKGS[@]}"
            else
                log "AUR packages already installed"
            fi
            ;;
        debian)
            sudo apt update
            sudo apt install -y \
                build-essential cmake pkg-config \
                libfreenect-dev libusb-1.0-0-dev libasound2-dev libpulse-dev \
                libopenni2-dev \
                qt6-base-dev qt6-tools-dev libopencv-dev \
                python3 python3-numpy python3-opencv
            ;;
        fedora)
            sudo dnf install -y \
                cmake gcc make pkg-config \
                libusb1-devel alsa-lib-devel pulseaudio-libs-devel \
                openni2-devel \
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
            local DEFAULT_FW="$HOME/Descargas/KinectRuntime-v1.8-Setup.exe"
            if [ -f "$DEFAULT_FW" ]; then
                read -p "Path to KinectRuntime-v1.x-Setup.exe [$DEFAULT_FW]: " exe_path
                exe_path="${exe_path:-$DEFAULT_FW}"
            else
                read -p "Path to KinectRuntime-v1.x-Setup.exe: " exe_path
            fi
            if [ ! -f "$exe_path" ]; then
                err "File not found: $exe_path"
                return 1
            fi
            info "Extracting firmware from $exe_path..."
            # Ensure dependencies
            for tool in cabextract msiextract; do
                if ! command -v "$tool" &>/dev/null; then
                    warn "$tool not found. Installing..."
                    case $DISTRO in
                        arch)    sudo pacman -S --noconfirm --needed msitools ;;
                        debian)  sudo apt install -y msitools ;;
                        fedora)  sudo dnf install -y msitools ;;
                    esac
                fi
            done
            tmpdir=$(mktemp -d)
            # Extract CAB from NSIS installer (CAB at offset 309248)
            dd if="$exe_path" of="$tmpdir/kr.cab" bs=1 skip=309248 2>/dev/null
            if ! file "$tmpdir/kr.cab" | grep -q "Cabinet"; then
                err "Could not extract CAB from installer."
                rm -rf "$tmpdir"
                return 1
            fi
            # Extract MSI from CAB (cabextract writes to CWD, so cd there)
            cd "$tmpdir"
            cabextract -F a2 "kr.cab" 2>/dev/null
            # Rename to .msi and extract
            cp a2 a2.msi
            msiextract a2.msi 2>/dev/null
            cd - >/dev/null
            # Find and install firmware
            fw=$(find "$tmpdir" -name "UAC.bin" -o -name "audios.bin" 2>/dev/null | head -1)
            if [ -n "$fw" ]; then
                sudo cp "$fw" "$FW_PATH"
                log "Firmware installed to $FW_PATH"
            else
                err "Could not find UAC.bin in installer. Try manual download."
            fi
            rm -rf "$tmpdir"
            ;;
        2)
            local FW_URL="https://www.microsoft.com/en-us/download/details.aspx?id=40277"
            local FW_DIRECT="https://download.microsoft.com/download/e/c/5/ec50686b-82f4-4dbf-a922-980183b214e6/KinectRuntime-v1.8-Setup.exe"
            info "Opening Microsoft download page..."
            # Try multiple browser launchers
            if command -v xdg-open &>/dev/null && xdg-open "$FW_URL" 2>/dev/null; then
                true
            elif command -v gnome-open &>/dev/null && gnome-open "$FW_URL" 2>/dev/null; then
                true
            elif command -v firefox &>/dev/null; then
                firefox "$FW_URL" &
            elif command -v chromium &>/dev/null; then
                chromium "$FW_URL" &
            elif command -v google-chrome &>/dev/null; then
                google-chrome "$FW_URL" &
            else
                warn "Could not open browser."
            fi
            echo ""
            echo "Download KinectRuntime-v1.8-Setup.exe (NOT KinectDeveloperToolkit)"
            echo "  Page:  $FW_URL"
            echo "  Direct: $FW_DIRECT"
            echo "Run this script again after downloading."
            exit 0
            ;;
        3)
            warn "Skipping firmware. Motor/LED won't work."
            ;;
    esac
}

# ─── Build ────────────────────────────────────────
build() {
    # Pre-built release — skip cmake/make unless source exists
    if [ -f "$SCRIPT_DIR/CMakeLists.txt" ]; then
        info "Building Kinect for Linux..."
        mkdir -p "$BUILD_DIR"
        cd "$BUILD_DIR"
        cmake "$SCRIPT_DIR" \
            -DCMAKE_BUILD_TYPE=Release
        make -j$(nproc)
        log "Build complete"
    else
        log "Pre-built release — skipping build"
    fi
}

# ─── Install ──────────────────────────────────────
install_app() {
    info "Installing Kinect for Linux..."
    if [ -f "$BUILD_DIR/CMakeCache.txt" ]; then
        cd "$BUILD_DIR"
        sudo cmake --install .
    else
        # Install pre-built binaries manually
        sudo install -Dm755 "$SCRIPT_DIR/k4wd" /usr/local/bin/k4wd.bin
        sudo install -Dm755 "$SCRIPT_DIR/k4wd-wrapper" /usr/local/bin/k4wd
        sudo install -Dm755 "$SCRIPT_DIR/libK4WDriver.so" /usr/local/lib/libK4WDriver.so
        sudo install -Dm755 "$SCRIPT_DIR/kinect-for-linux" /usr/local/bin/kinect-for-linux
        sudo install -Dm644 "$SCRIPT_DIR/kinect-for-linux.png" /usr/share/pixmaps/kinect-for-linux.png
        sudo install -Dm644 "$SCRIPT_DIR/kinect-for-linux.desktop" /usr/share/applications/kinect-for-linux.desktop
    fi

    # udev rules
    sudo cp "$SCRIPT_DIR/90-kinect.rules" /etc/udev/rules.d/
    sudo cp "$SCRIPT_DIR/51-kinect.rules" /etc/udev/rules.d/
    sudo cp "$SCRIPT_DIR/99-kinect-camera.rules" /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger

    # v4l2loopback config
    sudo cp "$SCRIPT_DIR/v4l2loopback.conf" /etc/modprobe.d/

    # WirePlumber config — disable Kinect USB audio for exclusive access
    sudo mkdir -p /etc/wireplumber/wireplumber.conf.d
    sudo cp "$SCRIPT_DIR/wireplumber/51-kinect-audio.conf" /etc/wireplumber/wireplumber.conf.d/

    # Blacklist gspca_kinect (conflicts with libfreenect USB access)
    if [ ! -f /etc/modprobe.d/blacklist-gspca-kinect.conf ]; then
        echo "blacklist gspca_kinect" | sudo tee /etc/modprobe.d/blacklist-gspca-kinect.conf >/dev/null
        sudo rmmod gspca_kinect 2>/dev/null || true
        sudo rmmod gspca_main 2>/dev/null || true
        log "Blacklisted gspca_kinect module"
    fi

    # Load v4l2loopback
    sudo modprobe v4l2loopback devices=1 video_nr=100 exclusive_caps=1 card_label="Kinect" 2>/dev/null || true

    # Install systemd user service
    mkdir -p "$HOME/.config/systemd/user"
    cp "$SCRIPT_DIR/k4wd.service" "$HOME/.config/systemd/user/"
    systemctl --user daemon-reload 2>/dev/null || true
    systemctl --user enable k4wd.service 2>/dev/null || true
    log "Systemd service installed (k4wd.service)"

    # Enable arrpc for Discord Rich Presence (if installed)
    if systemctl --user list-unit-files arrpc.service &>/dev/null; then
        systemctl --user enable arrpc.service 2>/dev/null || true
        log "Discord RPC service enabled (arrpc.service)"
    fi

    # Install ONNX models for C++ skeleton tracking
    sudo mkdir -p /usr/local/share/k4w-models
    local MODEL_DIR="/usr/local/share/k4w-models"
    local DETECT_MODEL="person_detection_mediapipe_2023mar.onnx"
    local POSE_MODEL="pose_estimation_mediapipe_2023mar_onnx.onnx"
    local BASE_URL="https://github.com/opencv/opencv_zoo/raw/main/models/pose_estimation_mediapipe"

    if [ ! -f "$MODEL_DIR/$DETECT_MODEL" ]; then
        info "Downloading person detection model..."
        sudo wget -q -O "$MODEL_DIR/$DETECT_MODEL" \
            "$BASE_URL/$DETECT_MODEL" 2>/dev/null || warn "Download failed: $DETECT_MODEL"
    else
        log "Detection model already installed"
    fi

    if [ ! -f "$MODEL_DIR/$POSE_MODEL" ]; then
        info "Downloading pose estimation model..."
        sudo wget -q -O "$MODEL_DIR/$POSE_MODEL" \
            "$BASE_URL/$POSE_MODEL" 2>/dev/null || warn "Download failed: $POSE_MODEL"
    else
        log "Pose model already installed"
    fi

    log "Models directory: $MODEL_DIR"

    # Setup OpenNI2 driver (symlink libFreenectDriver to OpenNI2 drivers dir)
    if [ -d /usr/lib/OpenNI2-FreenectDriver ] && [ -d /usr/lib/OpenNI2/Drivers ]; then
        sudo ln -sf /usr/lib/OpenNI2-FreenectDriver/libFreenectDriver.so /usr/lib/OpenNI2/Drivers/libFreenectDriver.so 2>/dev/null
        log "OpenNI2 FreenectDriver linked"
    fi

    log "Installation complete!"
    echo ""
    echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"
    echo -e "${GREEN} Kinect for Linux installed successfully!${NC}"
    echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"
    echo ""
    echo "  Daemon:         systemctl --user start k4wd"
    echo "  Daemon status:  systemctl --user status k4wd"
    echo "  Daemon logs:    journalctl --user -u k4wd -f"
    echo "  Run GUI:        kinect-for-linux"
    echo "  Audio sink:     'k4w-mic' (Monitor of k4w-mic)"
    echo "  Discord RPC:    systemctl --user start arrpc"
    echo "  Camera:         Restart Discord after daemon starts"
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
