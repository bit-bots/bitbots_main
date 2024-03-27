#!/bin/bash

# You need to fill out a form to download the pylon driver.
# The pylon driver can be found in the download section of the following link:
# https://www.baslerweb.com/en/downloads/software-downloads/
# Go to the download button and copy the link address.
PYLON_DOWNLOAD_URL="https://www2.baslerweb.com/media/downloads/software/pylon_software/pylon_7_4_0_14900_linux_x86_64_debs.tar.gz"
PYLON_VERSION="7.4.0"

# Check let the user confirm that they read the license agreement on the basler website and agree with it.
echo "You need to confirm that you read the license agreements for pylon $PYLON_VERSION on the basler download page (https://www.baslerweb.com/en/downloads/software-downloads/) and agree with it."

# Check --ci flag for automatic confirmation in the ci
if [[ $1 == "--ci" ]]; then
    echo "Running in a CI environment, continuing..."
    SHOW_PROGRESS=""
else
    # Ask the user if they want to continue and break if they don't
    read -p "Do you want to continue? [y/N] " -n 1 -r

    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Aborting..."
        exit 1
    fi
    SHOW_PROGRESS="--show-progress"
fi

# Create function to check if we have an internet connection
function check_internet_connection () {
    # Check if we have an internet connection, except in the ci as azure does not support ping by design
    if [[ $1 != "--ci" ]] && ! ping -q -c 1 -W 1 google.com >/dev/null; then
        echo "No internet connection. Please check your internet connection to install the basler drivers."
        exit 1
    fi
}

# Check if the correct pylon driver PYLON_VERSION is installed (apt)
if apt list pylon --installed | grep -q $PYLON_VERSION; then
    echo "Pylon driver $PYLON_VERSION is already installed."
else
    echo "Pylon driver $PYLON_VERSION is not installed. Installing..."
    # Check if we have an internet connection
    check_internet_connection $1
    # Check if the url exist
    if ! curl --output /dev/null --silent --head --fail "$PYLON_DOWNLOAD_URL"; then
        echo "Pylon download url does not exist. Please check the url and update the 'PYLON_DOWNLOAD_URL' variable in the 'make_basler.sh' script. The website might have changed."
        exit 1
    fi
    # Download the pylon driver to temp folder
    wget --no-verbose $SHOW_PROGRESS $PYLON_DOWNLOAD_URL -O /tmp/pylon_${PYLON_VERSION}.tar.gz
    # Extract the pylon driver
    mkdir /tmp/pylon
    tar -xzf /tmp/pylon_${PYLON_VERSION}.tar.gz -C /tmp/pylon/
    # Install the pylon driver
    sudo apt-get install /tmp/pylon/pylon_${PYLON_VERSION}*.deb -y
fi
