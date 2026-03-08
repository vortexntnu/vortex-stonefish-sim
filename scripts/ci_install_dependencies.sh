#!/bin/bash
set -e

STONEFISH_DIR="$(find "${BASEDIR}/${PREFIX}upstream_ws" -maxdepth 3 -type d -name stonefish | head -n 1)"

test -n "$STONEFISH_DIR"

apt-get update -qq
apt-get install -y --no-install-recommends software-properties-common
add-apt-repository ppa:ubuntu-toolchain-r/test -y
apt-get update -qq

apt-get install -y --no-install-recommends gcc-13 g++-13 lcov
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100
update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100
update-alternatives --install /usr/bin/gcov gcov /usr/bin/gcov-13 100

apt-get update -qq
apt-get install -y \
    build-essential \
    cmake \
    git \
    libglm-dev \
    libsdl2-dev \
    libfreetype6-dev

mkdir -p "$STONEFISH_DIR/build"
cd "$STONEFISH_DIR/build"

cmake ..
make -j"$(nproc)"
make install
