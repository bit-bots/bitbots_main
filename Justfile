# Justfile

# Set shell
set shell := ["bash", "-uc"]

# Variables
HTTPS := ""
REPO := `pwd`
CI := ""

# Setup / Update related targets

# Install the proprietary Basler Camera SDK
install-basler:
    scripts/make_basler.sh {{ if CI != "" {"--ci"} else {""} }}

# Install Webots Simulation environment
install-webots:
    scripts/make_webots.sh

# Install all dependencies, pull all repositories, ...
install: pull-init install-basler install-webots update

# Same as install but without sudo commands (apt/ros dependencies must be already installed on the system)
install-no-root: pull-init update-no-root

# Install python dependencies
pip:
    # Install and upgrade pip dependencies
    pip install --upgrade pip --user --break-system-packages -v
    pip install --upgrade -r requirements/dev.txt --user --break-system-packages -v


# Sets up pre-commit hooks that check code formatting etc. before each commit
pre-commit:
    pre-commit install

# This is needed as the ROS vscode extension adds absolute paths to some files and we do not want to commit those.
# Install git filters to remove absolute paths from files before committing them.
install-git-filters:
    # Install git filters
    git config filter.removeFullHomePath.clean "sed '/\/\(home\|root\).*\(install\|build\)/d'"

# Formats the code according to our style guidelines
format:
    pre-commit run --all-files

# Clones/Overrides all library repositories and pulls all auxiliary files (like neural network weights)
pull-init: fresh-libs pull-files

# Pull all auxiliary files (like neural network weights) from the http server
pull-files:
    wget --no-verbose --show-progress --timeout=15 --tries=2 --recursive --timestamping --no-parent --no-host-directories --directory-prefix={{REPO}}/src/bitbots_vision --reject "index.html*" "https://data.bit-bots.de/models/"
    wget --no-verbose --show-progress --timeout=15 --tries=2 --recursive --timestamping --no-parent --no-host-directories --directory-prefix={{REPO}}/src/bitbots_motion/bitbots_rl_motion --reject "index.html*" "https://data.bit-bots.de/rl_walk_models/"

# Remove all library repositories and re-clone them
fresh-libs: remove-libs clone-libs

# Remove all library repositories
remove-libs:
    rm -rf src/lib/*
    rm -rf src/bitbots_team_communication/bitbots_team_communication/bitbots_team_communication/RobocupProtocol
    rm src/bitbots_team_communication/bitbots_team_communication/robocup_extension_pb2.py || true

# Clone all library repositories
clone-libs:
    if [ "{{HTTPS}}" = "true" ]; then \
        awk '{sub("git@github.com:", "https://github.com/"); print "  " $$0}' workspace.repos | vcs import .; \
    else \
        vcs import . < workspace.repos; \
    fi

# Install all ROS managed dependencies
rosdep:
    [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

# Update everything (download all lib repositories, update dependencies, ...)
update: pull-init rosdep pip install-git-filters

# Same as update but without sudo commands (all apt/ros dependencies must be already installed on the system)
update-no-root: pull-init pip install-git-filters


# Build related targets

# Build a specific package or all packages
build *packages:
    colcon build --symlink-install --continue-on-error {{ if packages != "" {"--packages-select " + packages} else {""} }}

# Clean all build artifacts or only those of specific packages
clean *packages:
	if [ "{{packages}}" != "" ]; then \
		colcon clean packages --packages-select {{packages}}; \
	else \
		rm -rf install build log; \
	fi

# Run tests of all packages or a specific package
test *packages:
    colcon test --event-handlers console_direct+ --return-code-on-test-failure {{ if packages != "" {"--packages-select " + packages} else {""} }}

# Development related targets

# Deploy the code to a robot
deploy *args:
    scripts/deploy_robots.py {{args}}
