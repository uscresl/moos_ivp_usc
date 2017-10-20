#!/bin/bash

###############################################################################
# b2_5s
cp ../../../data/fake_bio/b2_5small.csv test.csv
scenario=b25s
########################################################################

# 1 auv lm
for idx in `seq 1 10`; do
	# start sim, 3auvs, area bigger2, warp 3, start from port 9000, lawnmower
	./launch.sh -n 3 -b bigger2 -w 3 -p 9000 -l |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 8000 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh curr_results/${scenario}_lm_3auvs_${idx}

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

# 1 auv adp
for idx in `seq 1 10`; do
	# start sim, 3 auvs, area bigger2, warp 3, start from port 9000
	./launch.sh -n 3 -b bigger2 -w 3 -p 9000 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 8000 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh curr_results/${scenario}_adp_3auvs_${idx}

	# clean up whatever remains
	sleep 5 && ./clean.sh
done
