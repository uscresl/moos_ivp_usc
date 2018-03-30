#!/bin/bash

###############################################################################
# b2_5s
#cp ../../../data/fake_bio/b2_5small.csv test.csv
#scenario=td
########################################################################

# meta_vehicle.moos has var_reduction
# 2 auv TDS, w/ shub, async trigger self-based var_reduction
# try for bigger area, ln -s used for test.csv, and trying bug fix
for idx in `seq 1 2`; do
  # start sim
	./launch.sh -n 2 -t -w 3 -p 8000 -b bigger2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
#	sleep 1400 && ./kill.sh # orig area: ~4k sec, 1400*3=4200
	sleep 4800 && ./kill.sh # bigger2: ~12.5k sec, 4400*3=13200

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh curr_results/async_20180329_b2_tds_wsh_var${idx}

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

sleep 10

# 2 auv TDS, w/ shub, timed
sed -i 's/"var_reduction"/"num_other_samples"/g' meta_vehicle.moos
# try for bigger area, ln -s used for test.csv, and trying bug fix
for idx in `seq 1 2`; do
  # start sim
	./launch.sh -n 2 -t -w 3 -p 8000 -b bigger2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
#	sleep 1400 && ./kill.sh # orig area: ~4k sec, 1400*3=4200
	sleep 4800 && ./kill.sh # bigger2: ~12.5k sec, 4400*3=13200

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh curr_results/async_20180329_b2_tds_wsh_NOS${idx}

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

sleep 10

## 2 auv TDS, w/ shub, timed
#sed -i 's/"num_other_samples"/"timed"/g' meta_vehicle.moos
## try for bigger area, ln -s used for test.csv, and trying bug fix
#for idx in `seq 1 2`; do
#  # start sim
#	./launch.sh -n 2 -t -w 3 -p 8000 -b bigger2 |&tee output.txt &
#	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

#	# wait for sim to run, then end it
##	sleep 1400 && ./kill.sh # orig area: ~4k sec, 1400*3=4200
#	sleep 4800 && ./kill.sh # bigger2: ~12.5k sec, 4400*3=13200

#	# move log files to dir
#	sleep 5 && ./mv_logs_to_dir.sh curr_results/async_20180329_b2_tds_wsh_timed${idx}

#	# clean up whatever remains
#	sleep 5 && ./clean.sh
#done

#sleep 10

## 2 auv TDS, no shub
## try for bigger area, ln -s used for test.csv, and trying bug fix
#for idx in `seq 1 2`; do
#  # start sim
#	./launch.sh -n 2 -t -w 3 -p 8000 -b bigger2 -d |&tee output.txt &
#	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

#	# wait for sim to run, then end it
##	sleep 1400 && ./kill.sh # orig area: ~4k sec, 1400*3=4200
#	sleep 4800 && ./kill.sh # bigger2: ~12.5k sec, 4400*3=13200

#	# move log files to dir
#	sleep 5 && ./mv_logs_to_dir.sh curr_results/async_20180329_b2_tds_nsh_${idx}

#	# clean up whatever remains
#	sleep 5 && ./clean.sh
#done

#sleep 10

## 2 auv VOR, no shub
## try for bigger area, ln -s used for test.csv, and trying bug fix
#for idx in `seq 1 2`; do
#  # start sim
#  ./launch.sh -n 2 -v -w 3 -p 8000 -b bigger2 -d |&tee output.txt &
#  sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

#  # wait for sim to run, then end it
##  sleep 1400 && ./kill.sh # orig area: ~4k sec, 1400*3=4200
#  sleep 4800 && ./kill.sh  # bigger2: ~12.5k sec, 4400*3=13200

#  # move log files to dir
#  sleep 5 && ./mv_logs_to_dir.sh curr_results/async_20180329_b2_vor_nsh_${idx}

#  # clean up whatever remains
#  sleep 5 && ./clean.sh
#done
