source /opt/ros/humble/setup.bash
rosdep update
cd /starline/ws_fastlio \
  && rosdep install --from-paths src --ignore-src -y \
  && colcon build --symlink-install \
  && source ./install/setup.bash
