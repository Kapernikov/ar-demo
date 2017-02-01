export parent_path=$(readlink -f ../)
xhost +
mkdir -p temp/eclipse
mkdir -p temp/workspace
docker run -it -e DISPLAY=$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v $HOME/.Xauthority:/root/.Xauthority \
       --rm -p 8888:8888 \
       -v $parent_path:/opt/source \
       -v $PWD/temp/eclipse:/root/.eclipse \
       -v $PWD/temp/workspace:/root/workspace \
       ros_with_python /bin/sh -c 'cd /opt/source/ros_catkin_ws; /opt/ros/kinetic/env.sh eclipse'

xhost -
