export parent_path=$(cd "../" 2>/dev/null && pwd -P)

ip=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
xhost + $ip
echo $ip
xhost +
mkdir -p temp/eclipse
mkdir -p temp/workspace
docker run -it -e DISPLAY=$ip:6000 \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
        \
       -v $HOME/.Xauthority:/root/.Xauthority \
       --rm  \
       -v $parent_path:/opt/source \
       -v $PWD/temp/eclipse:/root/.eclipse \
       -v $PWD/temp/workspace:/root/workspace \
       ros_standard /bin/sh -c 'cd /opt/source/ros_catkin_ws; /opt/ros/kinetic/env.sh bash -i -c eclipse'

xhost -
