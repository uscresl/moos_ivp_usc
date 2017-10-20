#!/bin/bash

# lm
for idx in `seq 1 1`; do
  # copy scenario
  cp ../../../data/fake_bio/human_pp_exp/field_$idx.csv test.csv
  
  # start sim
  ./launch.sh -ng --2d 3 |&tee output.txt &
  sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

  # wait for sim to run, then end it
  # 6460/3 = 2153.3333
  sleep 2500 && ./kill.sh

  # move log files to dir
  sleep 5 && ./mv_logs_to_dir.sh 1auv_lm_happ/lm_field_$idx

  # clean up whatever remains
  sleep 5 && ./clean.sh
done

# adp
for idx in `seq 1 12`; do
  # copy scenario
  cp ../../../data/fake_bio/human_pp_exp/field_$idx.csv test.csv
  
  # start sim
  ./launch.sh -a -ng --2d 3 |&tee output.txt &
  sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

  # wait for sim to run, then end it
  # 2600/3 = 866.66667
  sleep 1300 && ./kill.sh

  # move log files to dir
  sleep 5 && ./mv_logs_to_dir.sh 1auv_adp_happ/adp_field_$idx

  # clean up whatever remains
  sleep 5 && ./clean.sh
done

# lm
for idx in `seq 2 12`; do
  # copy scenario
  cp ../../../data/fake_bio/human_pp_exp/field_$idx.csv test.csv
  
  # start sim
  ./launch.sh -ng --2d 3 |&tee output.txt &
  sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

  # wait for sim to run, then end it
  # 6460/3 = 2153.3333
  sleep 2500 && ./kill.sh

  # move log files to dir
  sleep 5 && ./mv_logs_to_dir.sh 1auv_lm_happ/lm_field_$idx

  # clean up whatever remains
  sleep 5 && ./clean.sh
done


