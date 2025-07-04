#!/bin/bash
# This script will handle the uninstalation prosess.
# It will do any nessessary steps to guarenty a clean future instalation.


# Uninstallation
echo "Starting uninstalation..."
echo "Uninstalling testqt..."
cd ApexActual
make distclean
