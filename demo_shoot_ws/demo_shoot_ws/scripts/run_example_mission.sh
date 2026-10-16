#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_CMD="cd \"${WORKSPACE_DIR}\"; source /opt/ros/humble/setup.bash; if [ -f /usr/share/gazebo/setup.bash ]; then source /usr/share/gazebo/setup.bash; fi; source /opt/ican_shoot_sim/setup.bash; source \"${WORKSPACE_DIR}/install/setup.bash\""

gnome-terminal --window --title="sim" -e "bash -c \"${SETUP_CMD}; ros2 launch ican_shoot_sim sim.launch.py; exec bash\"" \
--tab --title="navigation" -e "bash -c \"sleep 5; ${SETUP_CMD}; ros2 launch ican_example_navigation navigation.launch.py; exec bash\"" \
--tab --title="example" -e "bash -c \"sleep 9; ${SETUP_CMD}; ros2 launch ican_example_mission one_target_demo.launch.py; exec bash\""
