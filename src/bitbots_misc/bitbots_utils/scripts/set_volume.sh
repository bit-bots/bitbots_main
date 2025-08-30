#!/bin/bash

if [[ -n $1 ]]; then
    amixer set Master $1
else
    amixer set Master 100%
fi >/dev/null
exit 0
