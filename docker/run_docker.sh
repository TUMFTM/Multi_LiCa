#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"

xhost +local:

docker run -it --rm --privileged \
    --network host \
    -e DISPLAY=${DISPLAY} \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $SCRIPT_DIR/../config:/ros_ws/src/multi_lidar_calibration/config \
    -v $SCRIPT_DIR/../data:/ros_ws/src/multi_lidar_calibration/data \
    -v $SCRIPT_DIR/../output:/ros_ws/src/multi_lidar_calibration/output \
    tum/ftm/multi_lidar_calibration:latest \
    /bin/bash

xhost -local:
