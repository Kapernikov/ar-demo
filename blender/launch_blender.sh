DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo $DIR
export PYTHONPATH=PYTHONPATH:$DIR
open -a /Applications/blender.app scene.blend