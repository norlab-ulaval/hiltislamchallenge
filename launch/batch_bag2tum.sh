script_dir=$(dirname $(readlink -f $0))
parent_dir="$(dirname "$script_dir")"

for file in $1/*.bag;
    do
    echo Processing $file;
    stem=$(basename -- "$file");
    ext="${stem##*.}";
    stem="${stem%.*}";

    ${parent_dir}/tum_export/bag2tum.sh --bagpath $file --vtk_filename $stem --imu_filt complementary
    # roslaunch hiltislamchallenge hilti_tmu.launch path:=$1 bagname:=$stem;
done
