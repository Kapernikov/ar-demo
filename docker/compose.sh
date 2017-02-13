export parent_path=$(cd "../" 2>/dev/null && pwd -P)
export COMPOSE_API_VERSION=auto

xhost +

export sourcedir=$parent_path
export webdir=$parent_path/blender
docker-compose build

docker-compose kill
docker-compose rm -f
docker-compose run interactive_shell


