#!/usr/bin/env bash

OFF='\033[0m'
RED='\033[0;31m'
GRN='\033[0;32m'
BLU='\033[0;34m'

BOLD=$(tput bold)
NORM=$(tput sgr0)

ERR="${RED}${BOLD}"
RRE="${NORM}${OFF}"

_usage="${BOLD}USAGE: scripts/setup_manus.sh DESIRED_WORKSPACE_PATH${NORM}
Installs the the MANUS VR Glove Packages into a DexHand workspace at
 (DESIRED_WORKSPACE_PATH)."

if [ $# -ne 1 ]; then
    echo -e "${ERR}ERROR: Incorrect number of arguments.${RRE}"
    echo -e "${_usage}"
    exit 1
fi

ROSDISTRO='humble'

# Clone the repositories to the src path
vcs import --input config/dexhand_manus.repos $1/src

# Set up ROS 2 Envrionment
echo -e "===================================================="
echo -e "Manus packages have been copied to the workspace."
echo -e "Please see the README.md in the manus_ros2 package for"
echo -e "instructions on how to install the Manus SDK and build the"
echo -e "workspace."
echo -e "https://github.com/iotdesignshop/manus_ros2"
echo -e "===================================================="



