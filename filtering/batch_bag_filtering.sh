bagfolder=$1

echo "Filtering bags in ${bagfolder}"

for file in $bagfolder/*.bag;
do
    echo Filtering $file;
    rosrun hiltislamchallenge filter_bag.sh $file;
done
