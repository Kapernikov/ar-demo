export parent_path=$(readlink -f ../)

docker run -it -e DISPLAY=$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v $HOME/.Xauthority:/root/.Xauthority \
       -v $parent_path:/opt/source \
       ros_with_python /bin/sh -c 'cd /opt/source/ros_catkin_ws; /opt/ros/kinetic/env.sh bash'

