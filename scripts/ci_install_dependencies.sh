#!/bin/bash
set -euo pipefail

apt-get update -qq
apt-get install -y --no-install-recommends \
    software-properties-common \
    build-essential \
    cmake \
    git \
    libglm-dev \
    libsdl2-dev \
    libfreetype6-dev \
    lcov

add-apt-repository ppa:ubuntu-toolchain-r/test -y
apt-get update -qq

apt-get install -y --no-install-recommends gcc-13 g++-13
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100
update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100
update-alternatives --install /usr/bin/gcov gcov /usr/bin/gcov-13 100

git clone --depth 1 https://github.com/vortexntnu/stonefish.git /tmp/stonefish

cmake -S /tmp/stonefish -B /tmp/stonefish/build
cmake --build /tmp/stonefish/build -j"$(nproc)"
cmake --install /tmp/stonefish/build
