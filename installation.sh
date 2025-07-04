#!/bin/bash
# This script will handle the instalation prosess.
# It will install all depedencies nessessary before doing any nessessary steps for a working app

# Depedencies
echo "Instaling depedencies..."
bash ./depedencies.sh

# Intallation
echo "Starting instalation..."
echo "Installing testqt..."
cd ApexActual
qmake qt_app.pro
make

