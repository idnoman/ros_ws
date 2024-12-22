#!/bin/bash

IMAGE_NAME="cpp_cameras"
PACKAGE_NAME="vision"
ROS_WS_DIR="/ros_ws"

# -e DISPLAY=$DISPLAY and -v /tmp/.X11... are only necessary to forward the graph session
docker run -it --rm --name test \
    --network host \
    -v $(pwd)/:"${ROS_WS_DIR}"/src/"${PACKAGE_NAME}" \
    --privileged \
    -v=/dev:/dev \
    --name "local" \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    "${IMAGE_NAME}" \
    bash -c "colcon build && source install/setup.bash && bash"
