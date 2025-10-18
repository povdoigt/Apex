#!/bin/bash

echo "Updating package list..."
sudo apt update

echo "Installing Qt5 development packages..."
sudo apt install -y qtbase5-dev qtwebengine5-dev libqt5webenginewidgets5 \
                    qttools5-dev-tools qtcreator qtwayland5 \
                    qtpositioning5-dev qtlocation5-dev \
                    libqt5webchannel5-dev qtmultimedia5-dev \
                    libqt5serialport5-dev libqt5x11extras5-dev

echo "Installing OpenStreetMap dependencies..."
sudo apt install -y libqt5positioning5 libqt5location5

echo "Installing X11 and Wayland support for WSL..."
sudo apt install -y libxcb-xinerama0 libxcb-icccm4 libxcb-image0 \
                    libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 \
                    libxcb-xkb1 libxkbcommon-x11-0 libegl1-mesa

echo "Installation complete. Restart WSL before using Qt."
