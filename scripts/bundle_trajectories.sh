scripts_dir=`rospack find hiltislamchallenge`/"scripts"

resultsdir=$scripts_dir/"results_offline_mapping"

trajsdir="$resultsdir/trajectories"
mkdir -p $trajsdir

for dir in $(ls -d -- $resultsdir/*/);
	do 
		name="$(basename $dir)";
		echo "Copying $name";
		tum_path="$resultsdir/$name/trajectories/traj/traj.txt"
		if [ -f $tum_path ]; then
			cp "$resultsdir/$name/trajectories/traj/traj.txt" $trajsdir/$name.txt;
		fi
done







