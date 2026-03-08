#!/usr/bin/env bash
# Installs dependencies not handled by rosdep.

set -e  # Exit on any error

# Note: These are required by vortex-vkf
# Add Ubuntu Toolchain PPA to get gcc-13/g++-13
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo apt-get update -qq

# Install and switch to GCC 13
sudo apt-get install -y --no-install-recommends gcc-13 g++-13 lcov
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100
sudo update-alternatives --install /usr/bin/gcov gcov /usr/bin/gcov-13 100

echo "Done installing additional dependencies."
