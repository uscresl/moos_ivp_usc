#!/bin/bash

# change acomms throughput to 100%
sed -i '/DROP_PCT=/c\DROP_PCT=0' launch.sh

#cp one_depth.csv test.csv

## voronoi
#for idx in `seq 1 10`; do
#	# start sim
#	./launch.sh --2auvs -a -t -v 2 |&tee output.txt &
#	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

#	# wait for sim to run, then end it
#	sleep 4000 && ./kill.sh

#	# move log files to dir
#	sleep 5 && ./mv_logs_to_dir.sh od_vor_$idx

#	# clean up whatever remains
#	sleep 5 && ./clean.sh
#done

#cp one_depth.csv test.csv

## no voronoi
#for idx in `seq 7 15`; do
#	# start sim
#	./launch.sh --2auvs -a -t 2 |&tee output.txt &
#	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

#	# wait for sim to run, then end it
#	sleep 4000 && ./kill.sh

#	# move log files to dir4
#	sleep 5 && ./mv_logs_to_dir.sh od_tds_$idx

#	# clean up whatever remains
#	sleep 5 && ./clean.sh
#done


cp two_depths.csv test.csv

# no voronoi
for idx in `seq 19 19`; do
	# start sim
	./launch.sh --2auvs -a -t 2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 4000 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh td_tds_$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

#cp two_depths.csv test.csv

## voronoi
#for idx in `seq 18 19`; do
#	# start sim
#	./launch.sh --2auvs -a -t -v 2 |&tee output.txt &
#	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

#	# wait for sim to run, then end it
#	sleep 4000 && ./kill.sh

#	# move log files to dir
#	sleep 5 && ./mv_logs_to_dir.sh td_vor_$idx

#	# clean up whatever remains
#	sleep 5 && ./clean.sh
#done


#cp one_depth_three.csv test.csv

## tds od3
#for idx in `seq 24 24`; do
#	# start sim
#	./launch.sh --2auvs -a -t 2 |&tee output.txt &
#	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

#	# wait for sim to run, then end it
#	sleep 4000 && ./kill.sh

#	# move log files to dir
#	sleep 5 && ./mv_logs_to_dir.sh od3_tds_$idx

#	# clean up whatever remains
#	sleep 5 && ./clean.sh
#done

cp one_depth_three.csv test.csv

# vor od3
for idx in `seq 17 17`; do
	# start sim
	./launch.sh --2auvs -a -t 2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 4000 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh od3_vor_$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done


#cp one_depth.csv test.csv

## no voronoi
#for idx in `seq 21 22`; do
#	# start sim
#	./launch.sh --2auvs -a -t 2 |&tee output.txt &
#	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

#	# wait for sim to run, then end it
#	sleep 4000 && ./kill.sh

#	# move log files to dir
#	sleep 5 && ./mv_logs_to_dir.sh od_tds_$idx

#	# clean up whatever remains
#	sleep 5 && ./clean.sh
#done

