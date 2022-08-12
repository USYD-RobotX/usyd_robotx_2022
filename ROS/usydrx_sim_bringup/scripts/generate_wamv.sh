#!/bin/bash
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"


roslaunch vrx_gazebo generate_wamv.launch thruster_yaml:=$SCRIPT_DIR/../wamv_config/thruster_config.yaml component_yaml:=$SCRIPT_DIR/../wamv_config/component_config.yaml wamv_target:=$SCRIPT_DIR/../urdf/usyd_vrx_wamv.urdf