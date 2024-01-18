.PHONY : basler install pip pre-commit pull-all pull-init pull-files rosdep status update

basler:
	scripts/make_basler.sh $(ARGS)

install: pull-init basler update

master:
	vcs import .. < workspace.repos

pip:
	# Install and upgrade pip dependencies
	pip install --upgrade -r requirements/dev.txt

pre-commit:
	# Install pre-commit hooks for all submodules that have a .pre-commit-config.yaml file
	pre-commit install

pull-all:
	git pull
	vcs pull ..
	scripts/pull_files.bash

pull-init:
	git pull
	vcs import .. < workspace.repos
	scripts/pull_files.bash

pull-files:
	scripts/pull_files.bash

rosdep:
	# Update rosdep and install dependencies from meta directory
	rosdep update
	rosdep install -iry --from-paths ..

status:
	vcs status ..

update: pull-all rosdep pip pre-commit
