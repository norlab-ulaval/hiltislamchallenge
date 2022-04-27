script_dir=$(dirname $(readlink -f $0))
parent_dir="$(dirname "$script_dir")"

run_bag2tum() {
    ${parent_dir}/tum_export/bag2tum.sh --bagpath $file --vtk_filename $stem --imu_filt $1
}

for file in $1/*.bag;
    do
    echo Processing $file;
    stem=$(basename -- "$file");
    ext="${stem##*.}";
    stem="${stem%.*}";

    run_bag2tum complementary
    # run_bag2tum madgwick

    # ${parent_dir}/tum_export/bag2tum.sh --bagpath $file --vtk_filename $stem --imu_filt madgwick
    # roslaunch hiltislamchallenge hilti_tmu.launch path:=$1 bagname:=$stem;
done
# for file in $1/*.bag;
#     do
#     echo Processing $file;
#     stem=$(basename -- "$file");
#     ext="${stem##*.}";
#     stem="${stem%.*}";

#     # run_bag2tum complementary
#     run_bag2tum madgwick

#     # ${parent_dir}/tum_export/bag2tum.sh --bagpath $file --vtk_filename $stem --imu_filt madgwick
#     # roslaunch hiltislamchallenge hilti_tmu.launch path:=$1 bagname:=$stem;
# done
