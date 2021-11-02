#!/bin/bash

###############################################################################
# b2_randomGauss field 8
ln -sf ../../../data/fake_bio/b2_randomGauss/rfield_8.csv test.csv

data_scenario=b2rg8
sim_date=20180404
########################################################################

# 2 auv TDS, w/ shub, async trigger self-based var_reduction
sed -i 's/async_trigger_method = "timed"/async_trigger_method = "var_reduction"/g' meta_vehicle.moos
# try for bigger area, ln -s used for test.csv, and trying bug fix
for idx in `seq 1 5`; do
  # start sim
	./launch.sh -n 2 -t -w 3 -p 8000 -b bigger2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
#	sleep 1400 && ./kill.sh # orig area: ~4k sec, 1400*3=4200
	sleep 5000 && ./kill.sh # bigger2: ~12.5k sec, 4400*3=13200

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh curr_results/async_${sim_date}_${data_scenario}_tds_wsh_var_a${idx}

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

sleep 10

# 2 auv TDS, w/ shub, timed
sed -i 's/"var_reduction"/"num_other_samples"/g' meta_vehicle.moos
# try for bigger area, ln -s used for test.csv, and trying bug fix
for idx in `seq 1 5`; do
  # start sim
	./launch.sh -n 2 -t -w 3 -p 8000 -b bigger2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
#	sleep 1400 && ./kill.sh # orig area: ~4k sec, 1400*3=4200
	sleep 4800 && ./kill.sh # bigger2: ~12.5k sec, 4400*3=13200

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh curr_results/async_${sim_date}_${data_scenario}_tds_wsh_NOS_a${idx}

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

sleep 10

# 2 auv TDS, w/ shub, timed
sed -i 's/"num_other_samples"/"timed"/g' meta_vehicle.moos
# try for bigger area, ln -s used for test.csv, and trying bug fix
for idx in `seq 1 5`; do
  # start sim
	./launch.sh -n 2 -t -w 3 -p 8000 -b bigger2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
#	sleep 1400 && ./kill.sh # orig area: ~4k sec, 1400*3=4200
	sleep 5000 && ./kill.sh # bigger2: ~12.5k sec, 4400*3=13200

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh curr_results/async_${sim_date}_${data_scenario}_tds_wsh_timed_a${idx}

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

sleep 10

# 2 auv TDS, no shub
# try for bigger area, ln -s used for test.csv, and trying bug fix
for idx in `seq 1 5`; do
  # start sim
	./launch.sh -n 2 -t -w 3 -p 8000 -b bigger2 -d |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
#	sleep 1400 && ./kill.sh # orig area: ~4k sec, 1400*3=4200
	sleep 4800 && ./kill.sh # bigger2: ~12.5k sec, 4400*3=13200

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh curr_results/async_${sim_date}_${data_scenario}_tds_nsh_a${idx}

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

sleep 10

# 2 auv VOR, no shub
# try for bigger area, ln -s used for test.csv, and trying bug fix
for idx in `seq 1 5`; do
  # start sim
  ./launch.sh -n 2 -v -w 3 -p 8000 -b bigger2 -d |&tee output.txt &
  sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

  # wait for sim to run, then end it
#  sleep 1400 && ./kill.sh # orig area: ~4k sec, 1400*3=4200
  sleep 4800 && ./kill.sh  # bigger2: ~12.5k sec, 4400*3=13200

  # move log files to dir
  sleep 5 && ./mv_logs_to_dir.sh curr_results/async_${sim_date}_${data_scenario}_vor_nsh_a${idx}

  # clean up whatever remains
  sleep 5 && ./clean.sh
done


###############################################################################
# b2_GPsamples field 1
ln -sf ../../../data/fake_bio/b2_GPsamples/sfield_1.csv test.csv

data_scenario=b2gp1
sim_date=20180405
########################################################################

# 2 auv TDS, w/ shub, async trigger self-based var_reduction
sed -i 's/async_trigger_method = "timed"/async_trigger_method = "var_reduction"/g' meta_vehicle.moos
# try for bigger area, ln -s used for test.csv, and trying bug fix
for idx in `seq 1 5`; do
  # start sim
	./launch.sh -n 2 -t -w 3 -p 8000 -b bigger2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
#	sleep 1400 && ./kill.sh # orig area: ~4k sec, 1400*3=4200
	sleep 5000 && ./kill.sh # bigger2: ~12.5k sec, 4400*3=13200

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh curr_results/async_${sim_date}_${data_scenario}_tds_wsh_var_a${idx}

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

sleep 10

# 2 auv TDS, w/ shub, timed
sed -i 's/"var_reduction"/"num_other_samples"/g' meta_vehicle.moos
# try for bigger area, ln -s used for test.csv, and trying bug fix
for idx in `seq 1 5`; do
  # start sim
	./launch.sh -n 2 -t -w 3 -p 8000 -b bigger2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
#	sleep 1400 && ./kill.sh # orig area: ~4k sec, 1400*3=4200
	sleep 4800 && ./kill.sh # bigger2: ~12.5k sec, 4400*3=13200

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh curr_results/async_${sim_date}_${data_scenario}_tds_wsh_NOS_a${idx}

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

sleep 10

# 2 auv TDS, w/ shub, timed
sed -i 's/"num_other_samples"/"timed"/g' meta_vehicle.moos
# try for bigger area, ln -s used for test.csv, and trying bug fix
for idx in `seq 1 5`; do
  # start sim
	./launch.sh -n 2 -t -w 3 -p 8000 -b bigger2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
#	sleep 1400 && ./kill.sh # orig area: ~4k sec, 1400*3=4200
	sleep 5000 && ./kill.sh # bigger2: ~12.5k sec, 4400*3=13200

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh curr_results/async_${sim_date}_${data_scenario}_tds_wsh_timed_a${idx}

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

sleep 10

# 2 auv TDS, no shub
# try for bigger area, ln -s used for test.csv, and trying bug fix
for idx in `seq 1 5`; do
  # start sim
	./launch.sh -n 2 -t -w 3 -p 8000 -b bigger2 -d |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

	# wait for sim to run, then end it
#	sleep 1400 && ./kill.sh # orig area: ~4k sec, 1400*3=4200
	sleep 4800 && ./kill.sh # bigger2: ~12.5k sec, 4400*3=13200

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh curr_results/async_${sim_date}_${data_scenario}_tds_nsh_a${idx}

	# clean up whatever remains
	sleep 5 && ./clean.sh
done

sleep 10

# 2 auv VOR, no shub
# try for bigger area, ln -s used for test.csv, and trying bug fix
for idx in `seq 1 5`; do
  # start sim
  ./launch.sh -n 2 -v -w 3 -p 8000 -b bigger2 -d |&tee output.txt &
  sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

  # wait for sim to run, then end it
#  sleep 1400 && ./kill.sh # orig area: ~4k sec, 1400*3=4200
  sleep 4800 && ./kill.sh  # bigger2: ~12.5k sec, 4400*3=13200

  # move log files to dir
  sleep 5 && ./mv_logs_to_dir.sh curr_results/async_${sim_date}_${data_scenario}_vor_nsh_a${idx}

  # clean up whatever remains
  sleep 5 && ./clean.sh
done
