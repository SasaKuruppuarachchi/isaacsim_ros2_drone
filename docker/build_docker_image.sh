#!/bin/bash
#docker rmi irlab-image
docker build --build-arg NUM_THREADS=20 --rm -t isaac-ros2-image .
