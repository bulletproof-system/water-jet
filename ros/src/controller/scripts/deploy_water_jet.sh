src=$(dirname $(dirname "$PWD"))
echo ${src}
ws=/home/robot/catkin_ws/src/fir_g3_morning
mkdir ${ws}
ln -s ${src}/controller ${ws}/controller
ln -s ${src}/create_map ${ws}/create_map
ln -s ${src}/nav_pkg ${ws}/nav_pkg
ln -s ${src}/world_simulation ${ws}/world_simulation