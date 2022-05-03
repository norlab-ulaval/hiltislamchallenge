for file in $1/*.bag;
    do
    echo Processing $file;
    stem=$(basename -- "$file");
    ext="${stem##*.}";
    stem="${stem%.*}";

    rosrun hiltislamchallenge bag2tum.py -b $file
    # roslaunch hiltislamchallenge hilti_tmu.launch path:=$1 bagname:=$stem;
done
