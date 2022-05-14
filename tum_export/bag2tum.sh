#!/bin/bash
# Call ./bag2tum.sh --bagpath /path/to/my/bagfile.bag --vtk_filename vtk_filename

# trap "exit" INT

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
bagstart=${start:-"0"}
bagduration=${duration:-"-1"}
description=${description:-""}

# Bagname path
path=${bagpath%/*}
stem=$(basename -- "$bagpath");
ext="${stem##*.}";
stem="${stem%.*}";

# Results path
pkg_path=`rospack find hiltislamchallenge`
results="$pkg_path"/"results"/"online"

bag_results="$results"/"$stem"
echo "Results folder: $bag_results"
if [ ! -d "$bag_results" ]; then
  mkdir -p "$bag_results"
fi

# Use stem for vtk_filename if not set yet
vtk_filename=${vtk_filename:-"$description"}

map_dir="$bag_results"/vtks
traj_dir="$bag_results"/trajectories
odom_dir="$bag_results"/odom
odombag_path=${odom_dir}/${stem}_${description}_odom.bag

echo "Map will be saved in $map_dir/${vtk_filename}_map.vtk"
echo "Traj will be saved in $map_dir/${vtk_filename}_traj.vtk"

mkdir -p $odom_dir
mkdir -p $map_dir
mkdir -p $traj_dir

start_time=`date +%s.%N`

roscore & sleep 2
# roslaunch hiltislamchallenge deskewed_rosbag_to_tum.launch path:=$path bagname:=$stem
rosparam set use_sim_time true
roslaunch hiltislamchallenge hilti_tmu.launch path:=$path bagname:=$stem imu_filter_type:=$imu_filt rate:=$rate start:=$bagstart duration:=$duration description:=$description &
sleep 3
while [[ ! -z `rosnode list | grep player` ]]
do
    sleep 1
done
killall rosbag_record_node

echo "MAPS : saving to $map_dir"
rosservice call /save_trajectory "trajectory_file_name:
   data: '$map_dir/${vtk_filename}_traj.vtk'"
rosservice call /save_map "map_file_name:
   data: '$map_dir/${vtk_filename}_map.vtk'"
echo "MAPS : Exported"
echo "----"
echo "ROSNODE : Killing nodes"
killall rviz
killall mapper_node
killall robot_state_publisher
killall imu_bias_compensation
killall imu_complementary_filter
killall imu_odom_node
killall pointcloud2_deskew_node
killall rosmaster
echo "ROSNODE : Killed nodes !"
echo "----"
# echo "TUM : saving to $traj_dir"
# echo "TUM : Odom Bag path $odombag_path"
# # rosrun hiltislamchallenge bag2tum.py -b $odombag_path
# echo "TUM: Exported"

# Kill the roscores
killall -9 rosmaster

end_time=`date +%s.%N`

runtime=$( echo "$end_time - $start_time" | bc -l )

times_path="$results"/times.dat
echo "$stem = $runtime" >> $times_path
