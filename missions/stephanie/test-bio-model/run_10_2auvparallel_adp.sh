#!/bin/bash

# run 10 simulations
for idx in `seq 1 12`; do
	# start sim
	./launch.sh --2auvs -a 5 &
	sleep 10 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 3600 && ./kill.sh

	# move log files to dir
	sleep 2 && ./mv_logs_to_dir.sh 0327_2auvsadp_parallel_$idx

	# clean up whatever remains
	sleep 2 && ./clean.sh
done


