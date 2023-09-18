.PHONY : basler install pip pre-commit pull-all pull-init pull-files rosdep status update

basler:
	scripts/make_basler.sh

install: pull-init
	scripts/make_update.sh

pip:
	scripts/make_pip.sh

pre-commit:
	scripts/make_pre-commit.sh

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
	scripts/make_rosdep.sh

status:
	scripts/git_status.bash

update: pull-all
	scripts/make_update.sh
