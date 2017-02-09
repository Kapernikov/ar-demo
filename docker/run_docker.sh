export parent_path=$(cd "../" 2>/dev/null && pwd -P)

xhost +

docker run -it --rm --name roscatkin -e DISPLAY=$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v $HOME/.Xauthority:/root/.Xauthority \
       -v $parent_path:/opt/source \
       ros_with_python /bin/bash -c '/opt/ros/kinetic/env.sh bash --rcfile /etc/bashrc_ros'

