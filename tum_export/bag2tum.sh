#!/bin/bash
# Call ./bag_2_tum.sh /path/to/my/bagfile.bag vtk_filename

bagpath=$1
path=${bagpath%/*}
stem=$(basename -- "$bagpath");
ext="${stem##*.}";
stem="${stem%.*}";

map_dir=$path/maps
traj_dir=$path/trajectories
odom_dir=$path/odom
odombag_path=${odom_dir}/${stem}_odom.bag
script_dir=`dirname $0`

mkdir -p $odom_dir
mkdir -p $map_dir
mkdir -p $traj_dir

roscore & sleep 2
# roslaunch hiltislamchallenge deskewed_rosbag_to_tum.launch path:=$path bagname:=$stem
rosparam set use_sim_time true
roslaunch hiltislamchallenge hilti_tmu.launch path:=$path bagname:=$stem

echo "MAPS : saving to $map_dir"
rosservice call /save_trajectory "trajectory_file_name:
   data: '$map_dir/$2_traj.vtk'"
rosservice call /save_map "map_file_name:
   data: '$traj_dir/$2_map.vtk'"
echo "MAPS : Exported"
echo "----"
echo "TUM : saving to $traj_dir"
echo "TUM : Odom Bag path $odombag_path"
${script_dir}/bag2tum.py -b $odombag_path
echo "TUM: Exported"

# Kill the roscores
killall -9 rosmaster



