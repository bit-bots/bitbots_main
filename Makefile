.PHONY : build doc doc-meta install repo vision-files update


build :
	scripts/build.sh repo vision-file
	scripts/install_py_extensions.bash
	scripts/repair.sh


doc :
	python3 scripts/build-complete-doc.py


doc-meta :
	python3 scripts/build-meta-doc.py


install: pull-all 
	scripts/install.pl

update: pull-all
	rosdep install -irya

pull-all:
	git pull
	scripts/pull_all.sh
	scripts/pull_files.bash


vision-files:
	scripts/pull_files.bash


