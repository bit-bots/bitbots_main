.PHONY : basler doc pip-install pre-commit pull-all pull-init rosdep-install status update

basler:
	test -d basler_drivers || git clone git@git.mafiasi.de:Bit-Bots/basler_drivers.git
	cd basler_drivers && ./setup.sh

doc:
	scripts/build-doc.py

pip-install:
	pip install --upgrade -r requirements/dev.txt

pre-commit:
	scripts/pre-commit-install.sh

pull-all:
	git pull
	scripts/pull_all.sh
	scripts/pull_files.bash

pull-init:
	git pull
	scripts/pull_init.sh
	scripts/pull_files.bash
	pre-commit-install

pull-files:
	scripts/pull_files.bash

rosdep-install:
	rosdep update
	rosdep install -iry --from-paths .

status:
	scripts/git_status.bash

update:
	pull-all
	rosdep-update
	pre-commit-install
	pip-install
