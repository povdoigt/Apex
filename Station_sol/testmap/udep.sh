#!/bin/bash

echo "Removing Qt5 development packages..."
sudo apt remove --purge -y qtbase5-dev qtwebengine5-dev libqt5webenginewidgets5 \
                           qttools5-dev-tools qtcreator qtwayland5 \
                           qtpositioning5-dev qtlocation5-dev \
                           libqt5webchannel5-dev qtmultimedia5-dev \
                           libqt5serialport5-dev libqt5x11extras5-dev

echo "Removing OpenStreetMap dependencies..."
sudo apt remove --purge -y libqt5positioning5 libqt5location5

echo "Removing X11 and Wayland dependencies..."
sudo apt remove --purge -y libxcb-xinerama0 libxcb-icccm4 libxcb-image0 \
                           libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 \
                           libxcb-xkb1 libxkbcommon-x11-0 libegl1-mesa

echo "Cleaning up..."
sudo apt autoremove -y
sudo apt autoclean

echo "Uninstallation complete."
