.PHONY : basler doc update pull-all pull-init status

basler:
	test -d basler_drivers || git clone git@git.mafiasi.de:Bit-Bots/basler_drivers.git
	cd basler_drivers && ./setup.sh

doc:
	scripts/build-doc.py

update: pull-all
	rosdep install -irya

pull-all:
	git pull
	scripts/pull_all.sh
	scripts/pull_files.bash

pull-init:
	git pull
	scripts/pull_init.sh
	scripts/pull_files.bash
	scripts/pre-commit-install.sh

pull-files:
	scripts/pull_files.bash

status:
	scripts/git_status.bash
