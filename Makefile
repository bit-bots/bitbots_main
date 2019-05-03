.PHONY : build doc doc-meta install repo vision-files update status


build :
	scripts/build.sh repo vision-file
	scripts/install_py_extensions.bash
	scripts/repair.sh

doc :
	python3 scripts/build-complete-doc.py

doc-meta :
	python3 scripts/build-meta-doc.py

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
