export parent_path=$(readlink -f ../)

docker run -it --rm -v $parent_path:/opt/source ros_with_python /bin/sh -c 'cd /opt/source/ros_catkin_ws; /opt/ros/kinetic/env.sh bash'

