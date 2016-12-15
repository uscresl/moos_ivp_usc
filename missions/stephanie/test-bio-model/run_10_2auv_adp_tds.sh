#!/bin/bash

# run 10 simulations
for idx in `seq 1 10`; do
	# start sim
	./launch.sh --2auvs -a -t 2 &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 4000 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh 20160717_2auvsadp_tds_10m_$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done
