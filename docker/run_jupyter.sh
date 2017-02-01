export parent_path=$(cd "../" 2>/dev/null && pwd -P)

docker run -it --rm -p 8888:8888 -e DISPLAY=$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v $HOME/.Xauthority:/root/.Xauthority \
       -v $parent_path:/opt/source \
       ros_with_python /bin/sh -c 'jupyter notebook --ip=*'

