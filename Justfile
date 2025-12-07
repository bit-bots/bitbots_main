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
