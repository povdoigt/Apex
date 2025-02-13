#!/bin/bash

echo "Updating package lists..."
sudo apt update

echo "Installing necessary dependencies..."
sudo apt install -y minicom screen g++ make

echo "Installation complete."
