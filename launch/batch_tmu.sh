for file in $1/*.bag;
    do
    echo Processing $file;
    stem=$(basename -- "$bagfile");
    ext="${stem##*.}";
    stem="${stem%.*}";
    roslaunch hiltislamchallenge hilti_tmu.launch path:=$1 bagname:=$stem;
done
