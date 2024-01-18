.PHONY : basler install pip pre-commit pull-all pull-init pull-files rosdep status update

HTTPS := ""

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
ifeq ($(HTTPS), true)
	awk '{sub("git@github.com:", "https://github.com/"); print "  " $$0}' workspace.repos | vcs import .. --skip-existing
else
	vcs import .. --skip-existing < workspace.repos
endif
	scripts/pull_files.bash

pull-files:
	scripts/pull_files.bash

rosdep:
	# Update rosdep and install dependencies from meta directory
	rosdep update
	# Small hack to make rosdep install all dependencies at once
	# See https://github.com/ros-infrastructure/rosdep/issues/671
	bash -c "sudo apt install -y $(rosdep check --from-paths .. --ignore-src --rosdistro iron | sed -n 's/^apt\s\+//p' | tr '\n' ' ')"

status:
	vcs status ..

update: pull-all rosdep pip pre-commit
