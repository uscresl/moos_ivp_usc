#!/bin/bash

# adp
for idx in `seq 1 30`; do
	# start sim
	./launch.sh -a -b2 -cg -cp -ng 3 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	# 15327/3 = 5109
	sleep 5300 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh 1auv_b24g/b24g_adp_1auv_p$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done
