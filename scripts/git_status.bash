#!/bin/bash

git submodule --quiet foreach 'unpushed=$(git log --oneline origin/HEAD..HEAD | wc -l); echo -ne "\033[1m$(basename $(realpath .)):\033[0m branch \033[1m$(git rev-parse --abbrev-ref HEAD)\033[0m, $(git status --porcelain | grep -c " M ") modified, $(git status --porcelain | grep -c "?? ") untracked"; [[ $unpushed != 0 ]] && echo -e ", \033[1m${unpushed} unpushed commit(s)\033[0m" || echo'
