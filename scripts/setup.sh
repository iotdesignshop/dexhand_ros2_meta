#!/usr/bin/env bash

OFF='\033[0m'
RED='\033[0;31m'
GRN='\033[0;32m'
BLU='\033[0;34m'

BOLD=$(tput bold)
NORM=$(tput sgr0)

ERR="${RED}${BOLD}"
RRE="${NORM}${OFF}"

_usage="${BOLD}USAGE: scripts/setup.sh DESIRED_WORKSPACE_PATH${NORM}
Installs the set of DexHand ROS 2 Packages into a ROS 2 Workspace 
at the specified path (DESIRED_WORKSPACE_PATH)."

if [ $# -ne 1 ]; then
    echo -e "${ERR}ERROR: Incorrect number of arguments.${RRE}"
    echo -e "${_usage}"
    exit 1
fi

ROSDISTRO='humble'

# Create a workspace directory
mkdir -p $1/src

# Clone the repositories to the src path
vcs import --input config/dexhand.repos $1/src

# Set up ROS 2 Envrionment
source /opt/ros/$ROSDISTRO/setup.bash

# Install dependencies
cd $1
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROSDISTRO

# This is specific for NVidia Orin / Jetpack because ROSDEP seems
# to be unable to find this package for Ubuntu20, 
# but should be harmless for other systems
sudo apt install -y ros-$ROSDISTRO-joint-state-publisher-gui

# Build the workspace
colcon build --symlink-install


