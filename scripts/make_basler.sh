#!/bin/bash
set -eEo pipefail

# You need to fill out a form to download the pylon driver.
# The pylon driver can be found in the download section of the following link:
# https://www.baslerweb.com/en/downloads/software-downloads/
# Go to the download button and copy the link address.
PYLON_DOWNLOAD_URL="https://data.bit-bots.de/pylon_7_4_0_14900_linux_x86_64_debs.tar.gz.gpg"
PYLON_VERSION="7.4.0"

CI=false
SKIP_OS_CHECK=false
SHOW_PROGRESS="--show-progress"
UNKNOWN_ARGS=()

print_help() {
    echo "Usage: $0 [--ci] [--skip-os-check] [--help]"
    echo
    echo "  --ci             Run non-interactively for CI (skips confirmation and progress)"
    echo "  --skip-os-check  Skip the Ubuntu version/OS check"
    echo "  -h, --help       Show this help message and exit"
}

# Parse arguments position-independently
for arg in "$@"; do
    case "$arg" in
        --ci)
            CI=true
            SHOW_PROGRESS=""
            ;;
        --skip-os-check)
            SKIP_OS_CHECK=true
            ;;
        -h|--help)
            print_help
            exit 0
            ;;
        *)
            UNKNOWN_ARGS+=("$arg")
            ;;
    esac
done

# If unknown arguments were passed, warn and show help
if [[ ${#UNKNOWN_ARGS[@]} -gt 0 ]]; then
    echo "Unknown argument(s): ${UNKNOWN_ARGS[*]}"
    echo
    print_help
    exit 1
fi

# Check let the user confirm that they read the license agreement on the basler website and agree with it.
echo "You need to confirm that you read the license agreements for pylon $PYLON_VERSION on the basler download page (https://www.baslerweb.com/en/downloads/software-downloads/) and agree with it."
# If not running in CI, ask the user to confirm license agreement
if [[ "$CI" != true ]]; then
    read -p "Do you want to continue? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Aborting..."
        exit 1
    fi
fi

check_os_is_required_ubuntu_version () {
    # Check if the OS is ubuntu
    echo "Checking for compatible OS..."
    if [[ "$(lsb_release -is)" != "Ubuntu" ]]; then
        echo "This driver package only supports Ubuntu (and some Debian derivatives)."
        echo "Please install Ubuntu >= 18.04 and try again OR try it on a compatible Debian derivative with --skip-os-check."
        exit 1
    fi
}

check_internet_connection () {
    # Check if we have an internet connection, except in the ci as azure does not support ping by design
    if [[ "$CI" != true ]] && ! ping -q -c 1 -W 1 google.com > /dev/null; then
        echo "No internet connection. Please check your internet connection to install the basler drivers."
        exit 1
    fi
}

# Check if the correct pylon driver PYLON_VERSION is installed (use dpkg-query)
INSTALLED_VERSION=$(dpkg-query -W -f='${Version}' pylon 2>/dev/null || true)
if [[ -n "$INSTALLED_VERSION" && "$INSTALLED_VERSION" == *"$PYLON_VERSION"* ]]; then
    echo "Pylon driver $PYLON_VERSION is already installed."
    exit 0
fi

echo "Pylon driver $PYLON_VERSION is not installed. Installing..."
# Check if the OS is the required ubuntu version
if [[ "$SKIP_OS_CHECK" != true ]]; then
    check_os_is_required_ubuntu_version
else
    echo "Skipping OS check."
fi

# Check if we have an internet connection
check_internet_connection
# Check if the url exist
if ! curl --output /dev/null --silent --head --fail "$PYLON_DOWNLOAD_URL"; then
    echo "Pylon download url does not exist. Please check the url and update the 'PYLON_DOWNLOAD_URL' variable in the 'make_basler.sh' script. The website might have changed."
    exit 1
fi
# Download the pylon driver to temp folder
wget --no-verbose $SHOW_PROGRESS $PYLON_DOWNLOAD_URL -O /tmp/pylon_${PYLON_VERSION}.tar.gz.gpg
# Extract the pylon driver
mkdir -p /tmp/pylon
# Decrypt the pylon driver
gpg --batch --yes --passphrase "12987318371043223" -o /tmp/pylon_${PYLON_VERSION}.tar.gz -d /tmp/pylon_${PYLON_VERSION}.tar.gz.gpg
# Extract the pylon driver
tar -xzf /tmp/pylon_${PYLON_VERSION}.tar.gz -C /tmp/pylon/
# Install the pylon driver
sudo apt-get install /tmp/pylon/pylon_${PYLON_VERSION}*.deb -y
