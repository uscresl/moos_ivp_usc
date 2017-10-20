#!/bin/bash

###############################################################################
# b2_5s
cp ../../../data/fake_bio/b2_5small.csv test.csv
########################################################################

# 1 auv lm
for idx in `seq 3 4`; do
	# start sim
	./launch.sh -n 1 -b bigger2 -w 3 -p 6000 -l -g |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 8000 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh curr_results/b25s_lm_1auv_$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

# 1 auv adp
for idx in `seq 1 30`; do
	# start sim
	./launch.sh -n 1 -b bigger2 -w 3 -p 6000 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 8000 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh curr_results/b25s_adp_1auv_$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done
