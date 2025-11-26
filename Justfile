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
	scripts/make_webots.sh {{ if CI != "" {"--ci"} else {""} }}


# Sets up pre-commit hooks that check code formatting etc. before each commit
install-pre-commit:
    pre-commit install

# This is needed as the ROS vscode extension adds absolute paths to some files and we do not want to commit those.
# Install git filters to remove absolute paths from files before committing them.
install-git-filters:
    # Install git filters
    git config filter.removeFullHomePath.clean "sed '/\/\(home\|root\).*\(install\|build\)/d'"

# Pull all auxiliary files (like neural network weights) from the http server
pull-files:
    wget --no-verbose --show-progress --timeout=15 --tries=2 --recursive --timestamping --no-parent --no-host-directories --directory-prefix={{REPO}}/src/bitbots_vision --reject "index.html*" "https://data.bit-bots.de/models/"
    wget --no-verbose --show-progress --timeout=15 --tries=2 --recursive --timestamping --no-parent --no-host-directories --directory-prefix={{REPO}}/src/bitbots_motion/bitbots_rl_motion --reject "index.html*" "https://data.bit-bots.de/rl_walk_models/"

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
