# Op Mac
## Build image
cd docker
./build_docker_ros_standard.sh

## Run container
Dit script start de container met -rm. Hierdoor zal de container niet blijven staan als ie stopt.
./run_docker_mac.sh

## In docker container
(Xephyr -ac -screen 1280x1024 -br -reset -terminate 2> /dev/null :1 &)
(Xephyr -ac -screen 1280x1024 :1 &)
Xephyr -ac -screen 1280x1024 -br -reset -terminate :2 &

Start een nieuwe terminal in de container (Aangezien de & de terminal precies niet terug geeft)
docker exec -it ros_standard_container bash -c "cd /opt/source/ros_catkin_ws; . devel/setup.bash; exec /bin/bash"

DISPLAY=:1 roslaunch ar_demo ar_demo.launch

./start-camera.sh aca1920_150uc_21917526