#!/bin/bash

env

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"

# Construct the full path to the 'world' file relative to the script's directory
# based on DNNMPC_LINEAR_SOLVER environment variable
if [ -z "${DNNMPC_LINEAR_SOLVER}" ] || [ "${DNNMPC_LINEAR_SOLVER}" = "default" ]; then
    # Default case: if DNNMPC_LINEAR_SOLVER is not set or set to 'default', use the "slow"
    # world file with RTF 0.1
    WORLD_FILE="$SCRIPT_DIR/worlds/dnn_mpc_ergoCubGazeboV1_1/world_rtf_0_1"
elif [ "${DNNMPC_LINEAR_SOLVER}" = "coinhsl" ]; then
    # If DNNMPC_LINEAR_SOLVER is set to 'coinhsl', use the "fast" world file with RTF 0.5
    WORLD_FILE="$SCRIPT_DIR/worlds/dnn_mpc_ergoCubGazeboV1_1/world_rtf_0_5"
else
    echo "Error: DNNMPC_LINEAR_SOLVER is set to an invalid value ('${DNNMPC_LINEAR_SOLVER}'). Valid values are 'default' or 'coinhsl'."
    exit 1
fi

# Start a new detached tmux session named 'dnn-mpc' with root directory '~/'
tmux new-session -d -s dnn-mpc -c ~/

# Rename ther first window to 'controller'
tmux rename-window -t dnn-mpc:0 controller

# Send the command to the 'controller' window's pan
tmux send-keys -t dnn-mpc:controller 'yarp wait /wholeBodyDynamics/left_foot_front/cartesianEndEffectorWrench:o ; YARP_COLORED_OUTPUT=1 YARP_ROBOT_NAME=ergoCubGazeboV1_1 YARP_CLOCK=/clock dnn-mpc-walking' C-m

# Create a new window named 'gazebo-server' with root directory '~/'
tmux new-window -t dnn-mpc: -n gazebo-server -c ~/

# Split the 'gazebo-server' window vertically to create two panes
tmux split-window -h -t dnn-mpc:gazebo-server -c ~/

# Set the layout of 'gazebo-server' window to 'main-vertical'
tmux select-layout -t dnn-mpc:gazebo-server main-vertical

# Send commands to each pane in the 'gazebo-server' window
tmux send-keys -t dnn-mpc:gazebo-server.0 "yarp wait /root ; gazebo -slibgazebo_yarp_clock.so '$WORLD_FILE'" C-m
tmux send-keys -t dnn-mpc:gazebo-server.1 'yarpserver --write' C-m

# Create a new window named 'whole-body-dynamics' with root directory '~/'
tmux new-window -t dnn-mpc: -n whole-body-dynamics -c ~/

# Send the command to the 'whole-body-dynamics' window's pane
tmux send-keys -t dnn-mpc:whole-body-dynamics 'yarp wait /ergocubSim/torso/state:o ; YARP_ROBOT_NAME=ergoCubGazeboV1_1 YARP_CLOCK=/clock yarprobotinterface --config conf/launch_wholebodydynamics_ecub.xml' C-m

# **Add these lines to select the desired window and pane before attaching**
# Select the window you want to focus on (e.g., 'controller', 'gazebo-server', or 'whole-body-dynamics')
tmux select-window -t dnn-mpc:controller

# Attach to the 'dnn-mpc' session
tmux attach-session -t dnn-mpc
