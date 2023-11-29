.PHONY : basler install pip pre-commit pull-all pull-init pull-files rosdep status update

basler:
	scripts/make_basler.sh $(ARGS)

install: pull-init basler update

master:
	git submodule foreach -q --recursive 'branch="$$(git config -f $$toplevel/.gitmodules submodule.$$name.branch)"; [ "$$branch" = "" ] && git switch -q master || git switch -q $$branch'

pip:
	# Install and upgrade pip dependencies
	pip install --upgrade -r requirements/dev.txt

pre-commit:
	# Install pre-commit hooks for all submodules that have a .pre-commit-config.yaml file
	git submodule foreach -q --recursive "test -f .pre-commit-config.yaml && pre-commit install || :"

pull-all:
	git pull
	scripts/pull_all.sh
	scripts/pull_files.bash

pull-init:
	git pull
	scripts/pull_init.sh
	scripts/pull_files.bash

pull-files:
	scripts/pull_files.bash

rosdep:
	# Update rosdep and install dependencies from meta directory
	rosdep update
	rosdep install -iry --from-paths .

status:
	scripts/git_status.bash

update: pull-all rosdep pip pre-commit
