#!/bin/bash

# change acomms throughput to 100%
sed -i '/DROP_PCT=/c\DROP_PCT=0' launch.sh

###############################################################################
# td: one bloom
cp two_depths.csv test.csv

################################################################################
# 3 auvs tds
for idx in `seq 1 5`; do
	# start sim
	./launch.sh --3auvs -a -t -ng 2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 2500 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh curr_results/td_tds_3auvs_$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

# 3 auvs vor
for idx in `seq 1 5`; do
	# start sim
	./launch.sh --3auvs -a -t -v -ng 2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 2500 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh curr_results/td_vor_3auvs_$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

