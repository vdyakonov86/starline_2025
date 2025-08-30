source /opt/ros/humble/setup.bash

mkdir -p /starline/ws_livox/src/Livox-SDK2/build \
  && cd /starline/ws_livox/src/Livox-SDK2/build \
  && cmake .. \
  && make -j \
  && sudo make install

cd /starline/ws_livox/src/livox_ros_driver2 \
  && ./build.sh humble \
  && source ../../install/setup.sh
