#!/bin/sh

# source this before you perform a distributed launch with a remote AutoRally chassis
export ROS_MASTER_URI=http://autorally:11311
export MASTER_HOSTNAME=autorally
export MASTER_USER=lbarnett
export HOSTNAME=$(hostname)
export ROSLAUNCH_SSH_UNKNOWN=1
# Find directory of script file to avoid hard-coded paths
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source ${DIR}/setupEnvVariables.sh