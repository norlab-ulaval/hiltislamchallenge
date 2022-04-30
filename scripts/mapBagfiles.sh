#! /bin/bash
# Use rosbag -u argument to speed up debugging on line 48
if [ -z $1 ]; then
    echo "Please set the folder containing rosbags as the first argument"
    exit
fi

sec=5
while [ $sec -ge 1 ]; do
  echo -ne "Killing rosmaster in $sec (ctrl+c to abort)\033[0K\r"
  let "sec=sec-1"
  sleep 1
done

killall roscore
killall rosmaster

data_folder=$1
results_folder="$(dirname -- "$(readlink -f "${BASH_SOURCE}")")"/results_offline_mapping
echo "Results folder: $results_folder"
if [ ! -d "$results_folder" ]; then
  mkdir -p "$results_folder"
fi

for bagfile in $(ls -v "$data_folder"/*.bag); do
  num_of_msgs=$(rosbag info $bagfile | grep '/hesai/pandar\s' | grep -oE '[[:digit:]]{3,9}')
  echo "Using rosbag $bagfile"
  echo "Expecting total of $num_of_msgs map messages"
  bagfile_file_name=${bagfile##*/}
  result_file="$results_folder"/"$bagfile_file_name"
  rm "$result_file.log"
  echo "Starting roscore"
  roscore >>"$result_file.log" &
  sleep 4
  echo "Setting use_sim_time to true"
  rosparam set /use_sim_time true

  echo "Starting offline_mapping_monitor"
  rosrun hiltislamchallenge offline_mapping_monitor.py _bagfile:=$bagfile 1>>"$result_file.log" 2>&1 &
  sleep 4
  echo "Starting hilti_offline_mapping.launch"
  roslaunch hiltislamchallenge hilti_offline_mapping.launch bagfile:="$bagfile" result_file_name:="$result_file" 1>>"$result_file.log" 2>&1 &
  roslaunch_job=$!
  echo "hilti_offline_mapping launched with job number $roslaunch_job"

  sleep 3

  echo "Starting rosbag"
  rosbag play $bagfile --clock --rate 1 -k --quiet -u 3 >>"$result_file.log" &
  rosbag_job=$!
  echo "Rosbag started with job number $rosbag_job"
  sleep 3

  while [[ ! -z $(pgrep mapper_node) ]]; do
    sleep 1
  done

  for rosnode in $(rosnode list); do
    rosnode kill "$rosnode"
  done
  killall -9 roscore
  killall -9 rosmaster

done
