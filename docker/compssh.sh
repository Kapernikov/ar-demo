export parent_path=$(cd "../" 2>/dev/null && pwd -P)
export COMPOSE_API_VERSION=auto

xhost +

export sourcedir=$parent_path
export webdir=$parent_path/blender
docker-compose -f docker-compose-ssh.yml build

docker-compose -f docker-compose-ssh.yml kill
docker-compose -f docker-compose-ssh.yml rm -f
docker-compose -f docker-compose-ssh.yml up



