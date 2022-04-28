bagfile=$1

mkdir -p filtered

echo Filtering ${bagfile}
# Remove camera topics
rosbag filter $bagfile filtered/${bagfile} "'image_raw' not in topic"
