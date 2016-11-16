#!/bin/bash

watch -n 1 "python ./test_kinematic.py 2>&1 | grep -v -e Util -e '^ *$' -e specialized"
