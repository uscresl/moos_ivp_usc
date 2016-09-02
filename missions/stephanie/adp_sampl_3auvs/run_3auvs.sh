#!/bin/bash

# change acomms throughput to 100%
sed -i '/DROP_PCT=/c\DROP_PCT=0' launch.sh

cp one_depth_three.csv test.csv

# run 2 simulation voronoi
for idx in `seq 1 2`; do
	# start sim
	./launch.sh --3auvs -a -t -v 2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 4000 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh od3_voronoi_3auvs_$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done
