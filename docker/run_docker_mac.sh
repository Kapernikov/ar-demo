export parent_path=$(cd "../" 2>/dev/null && pwd -P)

xhost +

DISNUM=$(echo $DISPLAY | cut -d : -f 2)
DISHOST=$(ipconfig getifaddr en0)

echo running with display $DISHOST:$DISNUM

# Dit creëert een container van image ros_standard.
# Deze container verdwijnt na stoppen van de machine:  --rm
# Zolang die leeft draagt ie de naam ros_standard_container
docker run -it --rm --name ros_standard_container -e DISPLAY=$DISHOST:$DISNUM \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v $HOME/.Xauthority:/root/.Xauthority \
       -v $parent_path:/opt/source \
       ros_standard /bin/bash -c '/opt/ros/kinetic/env.sh bash --rcfile /etc/bashrc_ros'

