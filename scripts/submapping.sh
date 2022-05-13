run_bag2tum() {
    rosrun hiltislamchallenge bag2tum.sh --bagpath $file --vtk_filename $stem --rate $1 --start $bagstart --duration $duration --description $description
}

select_parameters() {
    case $stem in
        exp09_cupola)
            echo "Case 09 - $stem";
            starts=(0 170 210 300 370);
            ends=(157 200 250 359 425);
            descriptions=("01scene" "02attic" "03cupola" "04stairscourt" "05scene");
            ;;
        exp11_lower_gallery)
            echo "Case 11 - $stem";
            starts=(0 130);
            ends=(119 "-1");
            descriptions=("01scene" "02return");
            ;;
        exp15_attic_to_upper_gallery)
            echo "Case 15 - $stem";
            starts=(0 83 165 220);
            ends=(80 160 218 260);
            descriptions=("01attic" "02gallery" "03stairsgarden" "04attic");
            ;;
        *)
            echo "Unknown sequence";
            exit 1;
            ;;
    esac
}

data_folder=$1

killall roscore
killall rosmaster

for file in $(ls -v "$data_folder"/exp{09,11,15}*.bag); do
    echo Processing $file;
    stem=$(basename -- "$file");
    ext="${stem##*.}";
    stem="${stem%.*}";

    select_parameters

    for idx in ${!starts[@]}; do
        bagstart=${starts[$idx]};
        bagend=${ends[$idx]};
        description=${descriptions[$idx]};
        duration=$( echo "$bagend - $bagstart" | bc -l )
        run_bag2tum 0.05
    done


    # run_bag2tum 0.05
    # run_bag2tum madgwick

    # ${parent_dir}/tum_export/bag2tum.sh --bagpath $file --vtk_filename $stem --imu_filt madgwick
    # roslaunch hiltislamchallenge hilti_tmu.launch path:=$1 bagname:=$stem;
done
