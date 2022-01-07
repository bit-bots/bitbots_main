#!/bin/zsh

export WEBOTS_HOME=${WEBOTS_HOME:-/usr/local/webots}
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${WEBOTS_HOME}/lib/controller
[[ -f /usr/lib/x86_64-linux-gnu/libtcmalloc_minimal.so.4 ]] && export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libtcmalloc_minimal.so.4
[[ -d ${WEBOTS_HOME}/lib/controller/python36 ]] && export PYTHONPATH=$PYTHONPATH:${WEBOTS_HOME}/lib/controller/python36
[[ -d ${WEBOTS_HOME}/lib/controller/python38 ]] && export PYTHONPATH=$PYTHONPATH:${WEBOTS_HOME}/lib/controller/python38
export PYTHONPATH=$PYTHONPATH:$( cd "$( dirname "$0" )" >/dev/null 2>&1 && pwd )
