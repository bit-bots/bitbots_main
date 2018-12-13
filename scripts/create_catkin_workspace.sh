#!/bin/bash

# Remember where we are
LOCATION="$(dirname $(dirname $(realpath $0)))"

# You can specify a location on the cli
if [[ -e $1 ]]; then
  WS_PATH=realpath $1
else
  WS_PATH="${HOME}/catkin_ws"
fi

# Check if a workspace already exists
if [[ -d $WS_PATH ]] ; then
  read -r -p "Workspace already exists at $WS_PATH. Overwrite? (y/N) " INPUT
  if [[ ! "$INPUT" =~ ^([yY][eE][sS]|[yY])+$ ]] ; then
    echo "aborting"
    exit
  fi
  echo "Removing old workspace"
  rm -rf "$WS_PATH"
fi

# Initialize the catkin workspace
echo "Initializing workspace at $WS_PATH"
source /opt/ros/kinetic/setup.bash
mkdir -p "${WS_PATH}/src"
cd "$WS_PATH"
catkin build

# Link bitbots_meta
ln -sf "$LOCATION" "${WS_PATH}/src/bitbots_meta"
echo "Workspace created at $WS_PATH"

# Give helpful cli advice
echo ""
echo "If this is your initial setup you include \"source $WS_PATH/devel/setup.bash\" in your \"~/.bashrc\""
read -r -p "Do you want me to do this for you? (Y/n) " INPUT
if [[ "$INPUT" =~ ^([yY][eE][sS]|[yY])+$ ]] ; then
  echo "source $WS_PATH/devel/setup.bash" >> $HOME/.bashrc
  echo "ok"
fi

echo ""
echo "You should call build.sh now to build from source"
 
