#!/bin/bash

echo "Updating package list..."
sudo apt update

echo "Installing Qt development libraries and tools..."
sudo apt install -y qtbase5-dev qtchooser qt5-qmake qttools5-dev-tools g++ pkg-config make

# Verify installations
echo "Verifying installation of Qt and build tools..."
if qmake -v &>/dev/null; then
    echo "qmake installed successfully"
else
    echo "qmake not found, please check installation logs."
fi

if g++ --version &>/dev/null; then
    echo "g++ installed successfully"
else
    echo "g++ not found, please check installation logs."
fi

if pkg-config --version &>/dev/null; then
    echo "pkg-config installed successfully"
else
    echo "pkg-config not found, please check installation logs."
fi

echo "All dependencies have been installed!"
