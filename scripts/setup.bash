source /opt/ros/humble/setup.bash
source /starline/ws_livox/install/setup.bash
source /starline/ws_fastlio/install/setup.bash
source /starline/ws_dev/install/setup.bash

export TURTLEBOT3_MODEL=waffle  # Iron and older only with Gazebo Classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models # Iron and older only with Gazebo Classic