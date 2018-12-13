#!/usr/bin/env bash
rm -rf ./env3/


virtualenv -p /usr/bin/python3 ./env3 || exit 1
source ./env3/bin/activate || exit 2

pip install -U -r requirements.txt

deactivate || exit 6
