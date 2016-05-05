#!/bin/bash

# run 10 simulations
for idx in `seq 1 10`; do
	# start sim
	./launch.sh --2auvs -a 2 &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 4000 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh 0329_2auvsadp_tds_$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done


