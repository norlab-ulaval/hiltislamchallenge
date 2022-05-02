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
results="$(dirname -- "$(readlink -f "${BASH_SOURCE}")")"/results_offline_mapping
echo "Results folder: $results"
if [ ! -d "$results" ]; then
  mkdir -p "$results"
fi

for bagfile in $(ls -v "$data_folder"/*.bag); do
  num_of_msgs=$(rosbag info $bagfile | grep '/hesai/pandar\s' | grep -oE '[[:digit:]]{3,9}')
  echo "Using rosbag $bagfile"
  echo "Expecting total of $num_of_msgs map messages"
  bagfile_file_name=${bagfile##*/}
  results_folder="$results"/"${bagfile_file_name%%.*}"_prior
  mkdir "$results_folder"
  log_file="$results_folder"/out.log
  mapping_file="$results_folder"/offline
  bagfile_rec_name="$results_folder"/traj
  echo "" >> "$log_file"
  echo "Starting roscore"
  roscore >>"$log_file" &
  sleep 4
  echo "Setting use_sim_time to true"
  rosparam set /use_sim_time true

  echo "Starting mapping_monitor"
  rosrun hiltislamchallenge mapping_monitor.py _bagfile:=$bagfile 1>>"$log_file" 2>&1 &
  sleep 4
  echo "Starting hilti_offline_mapping.launch"
  roslaunch hiltislamchallenge hilti_offline_mapping.launch bagfile_rec_name:="$bagfile_rec_name" result_file_name:="$mapping_file" 1>>"$log_file" 2>&1 &
  roslaunch_job=$!
  echo "hilti_offline_mapping launched with job number $roslaunch_job"

  sleep 3

  echo "Starting rosbag"
  rosbag play $bagfile --clock --rate 0.1 -k --quiet >>"$log_file" &
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

  rosrun hiltislamchallenge bag2tum.py --bagfile "$bagfile_rec_name".bag 1>>"$log_file" 2>&1

done
