#!/bin/bash

function echo_bold() {
    echo -ne "\033[1m$@\033[0m"
}

submodules=$(git submodule --quiet foreach 'pwd')
folders=". $submodules"

longest=0
for folder in $folders; do
    length=$(wc -c <<< $(basename $(realpath $folder)))
    if [[ $(( $length > $longest )) == 1 ]]; then
        longest=$length
    fi
done

for folder in $folders; do
    module=$(basename $(realpath $folder))
    length=$(wc -c <<< $module)
    branch=$(git -C $folder rev-parse --abbrev-ref HEAD)
    modified=$(git -C $folder status --ignore-submodules --porcelain | grep -c "^ M ")
    untracked=$(git -C $folder status --ignore-submodules --porcelain | grep -c "^?? ")
    unpushed=$(git -C $folder log --oneline origin/$branch..$branch | wc -l)
    echo_bold $module
    echo -n ":  "
    spaces=$(( $longest - $length ))
    while [[ $(( $spaces > 0 )) == 1 ]]; do
        echo -n " "
        spaces=$(( $spaces - 1 ))
    done
    echo -n "On branch $branch"
    [[ $modified != 0 ]] && echo -n ", $modified modified"
    [[ $untracked != 0 ]] && echo -n ", $untracked untracked"
    if [[ $unpushed == 1 ]]; then
        echo -n ", "
        echo_bold "1 unpushed commit"
    elif [[ $unpushed != 0 ]]; then
        echo -n ", "
        echo_bold "$unpushed unpushed commits"
    fi
    echo
done
