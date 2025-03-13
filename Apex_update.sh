#!/bin/bash
# This script is intended to be used when the app is being worked on.
# It assume that the git project was allready cloned at least once.
# It streamlines the update process by uninstalling the previous version, pulling the new version from git, and then installing it.

# Create variables for installation and uninstallation files for easy correction if nessessary
ufile="uninstallation.sh"
ifile="installation.sh"

# Uninstall previous installation to prevent potential errors
if [ -f "$ufile" ]; then  # Run the uninstallation script only if it exists
    echo "Starting uninstallation..."
    bash "$ufile"
    echo "Uninstallation done."
else
    echo "No uninstallation file found."
fi
    
# Pull the new version of the project from git
git pull

# Install the app
if [ -f "$ifile" ]; then  # Run the installation script only if it exists
    echo "Starting installation..."
    bash "$ifile"
    echo "Installation done."
else
    echo "No installation file found."
fi
