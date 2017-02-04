#!/bin/sh

docker exec -it roscatkin bash -c "cd /opt/source/ros_catkin_ws; . devel/setup.bash; exec /bin/bash"

