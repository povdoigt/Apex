#!/bin/bash

echo "Removing Qt development libraries and tools..."

# Remove Qt-related packages
sudo apt remove --purge -y qtbase5-dev qtchooser qt5-qmake qttools5-dev-tools g++ pkg-config make

# Clean up residual files
echo "Cleaning up unused dependencies..."
sudo apt autoremove -y
sudo apt clean

# Verify removal
echo "Checking if Qt and build tools are still present..."
if qmake -v &>/dev/null; then
    echo "Warning: qmake is still installed!"
else
    echo "qmake successfully removed."
fi

if g++ --version &>/dev/null; then
    echo "Warning: g++ is still installed!"
else
    echo "g++ successfully removed."
fi

if pkg-config --version &>/dev/null; then
    echo "Warning: pkg-config is still installed!"
else
    echo "pkg-config successfully removed."
fi

echo "Uninstallation process complete."
