#!/bin/bash
# Call ./bag_2_tum.sh /path/to/my/bagfile.bag vtk_filename

bagpath=$1
path=${bagpath%/*}
stem=$(basename -- "$bagpath");
ext="${stem##*.}";
stem="${stem%.*}";
map_path=$path/maps
traj_path=$path/trajectories

mkdir -p $map_path
mkdir -p $traj_path

roscore &
roslaunch hiltislamchallenge deskewed_rosbag_to_tum.launch path:=$path bagname:=$stem

echo "MAPS : saving to $map_path"
rosservice call /save_trajectory "trajectory_file_name:
   data: '$map_path/$2_traj.vtk'"
rosservice call /save_map "map_file_name:
   data: '$traj_path/$2_map.vtk'"
echo "MAPS : Exported"
echo "----"
echo "TUM : saving to $traj_path"
./bag_2_tum.py -b $bagpath
echo "TUM: Exported"

# Kill the roscores
killall -9 roscore
killall -9 rosmaster



