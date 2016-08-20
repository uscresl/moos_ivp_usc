#!/bin/bash

# change acomms throughput to 100%
sed -i '/DROP_PCT=/c\DROP_PCT=0' launch.sh

# run 10 simulation normal
for idx in `seq 1 10`; do
	# start sim
	./launch.sh --2auvs -a -t -v 2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 4000 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh tds_voronoi_a100_$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

# change acomms throughput to 70%
sed -i '/DROP_PCT=/c\DROP_PCT=30' launch.sh

# run 1 simulation 70%
for idx in `seq 1 10`; do
	# start sim
	./launch.sh --2auvs -a -t -v 2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
	sleep 4000 && ./kill.sh

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh tds_voronoi_a70_$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done
