#!/bin/bash

cd $(dirname $0)/../bitbots_vision/python_extensions/
python2 setup.py install --user
python3 setup.py install --user
