#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Need parameter: directory to move log files to"
  exit 0
fi

folder=$1

# change acomms throughput to 100%
sed -i '/DROP_PCT=/c\DROP_PCT=0' launch.sh

###############################################################################
# td: one bloom
cp two_depths.csv test.csv

################################################################################
# 3 auvs adp
for idx in `seq 1 10`; do
	# start sim
	./launch.sh -a -ng 2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 3600 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh ${folder}/td_1auv_adp_$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

# 3 auvs lm
for idx in `seq 1 10`; do
	# start sim
	./launch.sh -ng 2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 3600 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh ${folder}/td_1auv_lm_$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

