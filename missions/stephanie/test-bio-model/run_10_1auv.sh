#!/bin/bash

for idx in `seq 1 10`; do
	# start sim
	./launch.sh 5 &
	sleep 10 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait and kill
	sleep 3600 && ./kill.sh

	# store logs
	sleep 2 && ./mv_logs_to_dir.sh 0326_1auvlm_$idx

	# clean up rest
	sleep 2 && ./clean.sh
done

for idy in `seq 1 10`; do
	./launch.sh -a 5 &
	sleep 10 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	sleep 3600 && ./kill.sh

	sleep 2 && ./mv_logs_to_dir.sh 0326_1auvadp_$idy

	sleep 2 && ./clean.sh
done
