#!/bin/bash
docker run --rm -p 5900:5900 \
    -e HOME=/ \
    -v "$PWD":/home/rossim:rw \
    gazebo_ros:0.0 \
    x11vnc -forever -create
