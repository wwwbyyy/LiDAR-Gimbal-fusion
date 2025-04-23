#!/bin/bash

xhost + # for rviz

docker run -it --net=host --restart=always -e DISPLAY=$DISPLAY --privileged -v /tmp/.X11-unix:/tmp/.X11-unix -v [YOUR PATH]:/home/loc/lidar_gimbal_loc/ --name lidar_gimbal_loc_container lidar_gimbal_loc
