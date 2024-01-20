.PHONY : basler install pip pre-commit format pull-all pull-init pull-files fresh-libs remove-libs setup-libs rosdep status update

HTTPS := ""
REPO:=$(dir $(abspath $(firstword $(MAKEFILE_LIST))))

basler:
	# Install Basler Pylon SDK
	scripts/make_basler.sh $(ARGS)

install: pull-init basler update

pip:
	# Install and upgrade pip dependencies
	pip install --upgrade -r requirements/dev.txt

pre-commit:
	# Install pre-commit hooks for all submodules that have a .pre-commit-config.yaml file
	pre-commit install

format:
	# Format all files in the repository
	pre-commit run --all-files

pull-all:
	# Pull all repositories and untracked files
	vcs pull . --nested
	scripts/pull_files.bash

pull-init: fresh-libs pull-files

fresh-libs: remove-libs setup-libs

remove-libs:
	# Removes the lib directory and all its contents
	rm -rf lib/*

setup-libs:
	# Clone lib repositories in workspace.repos into the lib directory
ifeq ($(HTTPS), true)
	# Replace git@ with https:// to allow cloning without ssh keys
	awk '{sub("git@github.com:", "https://github.com/"); print "  " $$0}' workspace.repos | vcs import .
else
	vcs import . < workspace.repos
endif

pull-files:
	# Pull all large files (mainly neural network weights) from the http server
	wget \
		--no-verbose \
		--show-progress \
		--recursive \
		--timestamping \
		--no-parent \
		--no-host-directories \
		--directory-prefix=$(REPO)/bitbots_vision/bitbots_vision \
		--reject "index.html*" \
		"https://data.bit-bots.de/models/"
	wget \
		--no-verbose \
		--show-progress \
		--recursive \
		--timestamping \
		--no-parent \
		--no-host-directories \
		--directory-prefix=$(REPO)/bitbots_motion/bitbots_rl_motion \
		--reject "index.html*" \
		"https://data.bit-bots.de/rl_walk_models/"

rosdep:
	# Update rosdep and install dependencies from meta directory
	rosdep update
	# Small hack to make rosdep install all dependencies at once
	# See https://github.com/ros-infrastructure/rosdep/issues/671
	bash -c "sudo apt install -y $(rosdep check --from-paths . --ignore-src --rosdistro iron | sed -n 's/^apt\s\+//p' | tr '\n' ' ')"

status:
	# Show status of all repositories
	vcs status . --nested

update: pull-all rosdep pip pre-commit
