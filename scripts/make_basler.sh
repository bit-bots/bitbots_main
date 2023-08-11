#!/bin/sh

# Install Basler drivers
# Check if basler_drivers is already installed
# If not, clone the repository
# Run the setup script to install the drivers
test -d basler_drivers || git clone git@git.mafiasi.de:Bit-Bots/basler_drivers.git
cd basler_drivers && ./setup.sh
