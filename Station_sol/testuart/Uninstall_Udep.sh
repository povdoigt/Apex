#!/bin/bash

echo "Removing installed dependencies..."
sudo apt remove --purge -y minicom screen g++ make

echo "Cleaning up unused packages..."
sudo apt autoremove -y

echo "Uninstallation complete."
