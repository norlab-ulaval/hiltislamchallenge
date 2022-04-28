#!/bin/bash
# Call ./bag2tum.sh --bagpath /path/to/my/bagfile.bag --vtk_filename vtk_filename


# Pass named parameters in Bash
# https://brianchildress.co/named-parameters-in-bash/
while [ $# -gt 0 ]; do
    if [[ $1 == *"--"* ]]; then
        param="${1/--/}"
        declare $param="$2"
        echo "$1:$2" # Optional to see the parameter:value result
    fi
  shift
done

# Default parameters values
bagpath=${bagpath}
imu_filt=${imu_filt:-complementary}
rate=${rate:-"0.1"}

path=${bagpath%/*}
stem=$(basename -- "$bagpath");
ext="${stem##*.}";
stem="${stem%.*}";

# Use stem for vtk_filename if not set yet
vtk_filename=${vtk_filename:-$stem}

map_dir=$path/maps
traj_dir=$path/trajectories
odom_dir=$path/odom
odombag_path=${odom_dir}/${stem}_odom_${imu_filt}.bag
script_dir=`dirname $0`

mkdir -p $odom_dir
mkdir -p $map_dir
mkdir -p $traj_dir

roscore & sleep 2
# roslaunch hiltislamchallenge deskewed_rosbag_to_tum.launch path:=$path bagname:=$stem
rosparam set use_sim_time true
roslaunch hiltislamchallenge hilti_tmu.launch path:=$path bagname:=$stem imu_filter_type:=$imu_filt rate:=$rate

echo "MAPS : saving to $map_dir"
rosservice call /save_trajectory "trajectory_file_name:
   data: '$traj_dir/${vtk_filename}_traj.vtk'"
rosservice call /save_map "map_file_name:
   data: '$map_dir/${vtk_filename}_map.vtk'"
echo "MAPS : Exported"
echo "----"
echo "TUM : saving to $traj_dir"
echo "TUM : Odom Bag path $odombag_path"
${script_dir}/bag2tum.py -b $odombag_path
echo "TUM: Exported"

# Kill the roscores
killall -9 rosmaster



