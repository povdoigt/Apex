#!/bin/bash
# This script is intended to be used when the app is being worcked on.
# It streamline the uptate prosess by uninstalling the previous vertion, pulling from git the new vertion before installing it.
# While it is in the git file it expected to be one root above it to function correctly (e.g. : in the file "station" there is this shell scrpit and the file "Apex" witch is liked to the git)



# Clean previous install
# Specify the directory and the file name
dir="Apex"
ufile="uninstalation.sh"
ifile="instalation.sh"


if [ -f "$dir" ]; then # Check to see is Apex is allready cloned, I could not think of a way to make it worck without a previous clone.
    
    echo "Instance of Apex found"
    cd "$dir" || exit 1  # Change to the directory

    # Unistall previous instalation of the app to prevent potential error
    if [ -f "$ufile"] #I run the uninstalation file only if I find it to prevent error
        echo "Starting uninstalation..."
        bash ./uninstalation.sh
        echo "Uninstalation done."
    else
        echo "No unistall file found"
    fi
    
    # Pull the new vertion of the project from git
    git pull

    # Install the app
    if [ -f "$ifile"] #I run the instalation file only if I find it to prevent error
        echo "Starting instalation..."
        bash ./instalation.sh
        echo "Instalation done."
    else
        echo "No istall file found"
    fi
else
    echo "No instance of Apesex found do a first instalation manualy"
fi
