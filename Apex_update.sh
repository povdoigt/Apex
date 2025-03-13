#!/bin/bash
# This script is intended to be used when the app is being worked on.
# It streamlines the update process by uninstalling the previous version, pulling the new version from git, and then installing it.
# While it is in the git repository, it is expected to be one level above it for correct functionality (e.g., if the script is in the "station" directory, there is also an "Apex" directory linked to the git repository).

# Clean previous installation
# Specify the directory and the file name
dir="Apex"
ufile="uninstallation.sh"
ifile="installation.sh"

if [ -d "$dir" ]; then # Check if Apex directory is already cloned
    echo "Instance of Apex found"
    cd "$dir" || exit 1  # Change to the directory or exit if failed

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
else
    echo "No instance of Apesex found. Please perform the first installation manually."
fi
