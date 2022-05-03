#!/bin/bash
# Call ./bag2tum.sh --bagpath /path/to/my/bagfile.bag

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

path=${bagpath%/*}
stem=$(basename -- "$bagpath");
ext="${stem##*.}";
stem="${stem%.*}";

traj_dir=$path/trajectories
odom_dir=$path/odom
odombag_path=${odom_dir}/${stem}_odom_icp.bag

mkdir -p $odom_dir
mkdir -p $traj_dir

roscore & sleep 2
# rosbag play $path/tfs/tf_static.bag &&
rosparam set use_sim_time true
roslaunch hiltislamchallenge icp_to_tum.launch path:=$path bagname:=$stem

echo "----"
echo "TUM : saving to $traj_dir"
echo "TUM : Odom Bag path $odombag_path"
rosrun hiltislamchallenge bag2tum.py -b $odombag_path -t /icp_odom
echo "TUM: Exported"

# Kill the roscores
killall -9 rosmaster



