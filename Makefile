.PHONY : basler doc install update pull-all pull-init vision-files status

basler:
	test -d basler_drivers || git clone git@git.mafiasi.de:Bit-Bots/basler_drivers.git
	cd basler_drivers && ./setup.sh

doc:
	scripts/build-doc.py

install: pull-init
	scripts/install.pl

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

vision-files:
	scripts/pull_files.bash

status:
	scripts/git_status.bash
