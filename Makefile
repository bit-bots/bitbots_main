.PHONY : basler webots install install-no-root pip pre-commit install-git-filters format pull-all pull-init pull-repos pull-files fresh-libs remove-libs setup-libs rosdep status update update-no-root

HTTPS := ""
REPO:=$(dir $(abspath $(firstword $(MAKEFILE_LIST))))

basler:
	# Install Basler Pylon SDK
	scripts/make_basler.sh $(ARGS)

webots:
	# Install Webots Simulation environment
	scripts/make_webots.sh $(ARGS)

install: pull-init basler update

install-no-root: pull-init update-no-root

pip:
	# Install and upgrade pip dependencies
	pip install --upgrade pip --user
	pip install --upgrade -r requirements/dev.txt --user -v

pre-commit:
	# Install pre-commit hooks for all submodules that have a .pre-commit-config.yaml file
	pre-commit install

install-git-filters:
	# Install git filters
	# The vscode settings file gets updated by the ros extension and contains the full path to the current user's home directory.
	# We don't want to commit this path, so we use a git filter to remove it when git adds the file to the staging area.
	# This does not affect the file on disk, so vscode will still work as expected.
	git config filter.removeFullHomePath.clean "sed '/\/\(home\|root\).*\(install\|build\)/d'"

format:
	# Format all files in the repository
	pre-commit run --all-files

pull-all: pull-repos pull-files

pull-init: fresh-libs pull-files

pull-repos:
	# Pull all repositories
	vcs pull . --nested

pull-files:
	# Pull all large files (mainly neural network weights) from the http server
	wget \
		--no-verbose \
		--show-progress \
		--timeout=15 \
		--tries=2 \
		--recursive \
		--timestamping \
		--no-parent \
		--no-host-directories \
		--directory-prefix=$(REPO)/bitbots_vision \
		--reject "index.html*" \
		"https://data.bit-bots.de/models/"
	wget \
		--no-verbose \
		--show-progress \
		--timeout=15 \
		--tries=2 \
		--recursive \
		--timestamping \
		--no-parent \
		--no-host-directories \
		--directory-prefix=$(REPO)/bitbots_motion/bitbots_rl_motion \
		--reject "index.html*" \
		"https://data.bit-bots.de/rl_walk_models/"

fresh-libs: remove-libs setup-libs

remove-libs:
	# Removes the lib directory and all its contents
	rm -rf lib/*
	# Also remove the generated protobuf files, as they are not needed anymore
	rm bitbots_team_communication/bitbots_team_communication/robocup_extension_pb2.py 2> /dev/null || true

setup-libs:
	# Clone lib repositories in workspace.repos into the lib directory
ifeq ($(HTTPS), true)
	# Replace git@ with https:// to allow cloning without ssh keys
	awk '{sub("git@github.com:", "https://github.com/"); print "  " $$0}' workspace.repos | vcs import .
else
	vcs import . < workspace.repos
endif

rosdep:
	# Initialize rosdep if not already done
	[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
	# Update rosdep and install dependencies from meta directory
	rosdep update --include-eol-distros
	rosdep install --from-paths . --ignore-src --rosdistro humble -y

status:
	# Show status of all repositories
	vcs status . --nested

update: pull-all rosdep pip install-git-filters pre-commit

update-no-root: pull-all pip install-git-filters pre-commit
