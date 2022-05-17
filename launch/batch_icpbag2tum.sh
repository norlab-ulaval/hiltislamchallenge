run_bag2tum() {
    # echo File is $file;
    rosrun hiltislamchallenge icpbag2tum.sh --bagpath $file
}

for file in $1/*.bag;
    do
    echo Processing $file;
    stem=$(basename -- "$file");
    ext="${stem##*.}";
    stem="${stem%.*}";

    # run_bag2tum complementary
    run_bag2tum

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
