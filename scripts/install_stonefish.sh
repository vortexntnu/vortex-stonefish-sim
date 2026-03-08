#!/bin/bash
set -euo pipefail

STONEFISH_DIR="/opt/stonefish"

sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libglm-dev \
    libsdl2-dev \
    libfreetype6-dev

if [ -d "$STONEFISH_DIR" ]; then
    echo "Removing existing directory: $STONEFISH_DIR"
    sudo rm -rf "$STONEFISH_DIR"
fi

sudo git clone --depth 1 https://github.com/vortexntnu/stonefish.git "$STONEFISH_DIR"
sudo mkdir -p "$STONEFISH_DIR/build"
sudo chown -R "$USER:$USER" "$STONEFISH_DIR"

cd "$STONEFISH_DIR/build"

cmake ..
make -j"$(nproc)"
sudo make install
