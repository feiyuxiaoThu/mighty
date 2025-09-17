#!/bin/bash

source ~/.bashrc
source /home/code/dynus_ws/install/setup.bash
source /usr/share/gazebo/setup.sh

# (0) sim in static environment
python3 src/dynus/launch/run_single_sim.py --env easy_forest

# (1) sim in dynamic environment
# python3 src/dynus/launch/run_single_sim.py --env dynamic_forest