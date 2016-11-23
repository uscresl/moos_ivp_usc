#!/bin/bash

# change acomms throughput to 100%
sed -i '/DROP_PCT=/c\DROP_PCT=0' launch.sh

###############################################################################
# od: two blooms
cp one_depth.csv test.csv

################################################################################
# 3 auvs tds
for idx in `seq 1 5`; do
	# start sim
	./launch.sh --3auvs -a -t 2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 2000 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh od_tds_3auvs_l$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

# 3 auvs vor
for idx in `seq 1 5`; do
	# start sim
	./launch.sh --3auvs -a -t -v 2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 2000 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh od_vor_3auvs_l$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done


###############################################################################
# one bloom
cp two_depths.csv test.csv

###############################################################################
# 3 auvs tds
for idx in `seq 1 5`; do
	# start sim
	./launch.sh --3auvs -a -t 2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 2000 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh td_tds_3auvs_l$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

# 3 auvs vor
for idx in `seq 1 5`; do
	# start sim
	./launch.sh --3auvs -a -t -v 2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 2000 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh td_vor_3auvs_l$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

###############################################################################
# od: two blooms
cp one_depth.csv test.csv

################################################################################
# 3 auvs tds
for idx in `seq 1 5`; do
	# start sim
	./launch.sh --2auvs -a -t 2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 3600 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh od_tds_2auvs_l$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

# 3 auvs vor
for idx in `seq 1 5`; do
	# start sim
	./launch.sh --2auvs -a -t -v 2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 3600 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh od_vor_2auvs_l$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

