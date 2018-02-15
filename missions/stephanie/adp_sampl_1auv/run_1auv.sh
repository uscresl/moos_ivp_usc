#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Need parameter: directory to move log files to"
  exit 0
fi

folder=$1

################################################################################
# 1 AUV dyn prog run
for idx in `seq 1 30`; do
	# start sim
	./launch.sh 3
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 2400 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh ${folder}/td_dp_h10_nw_$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done
