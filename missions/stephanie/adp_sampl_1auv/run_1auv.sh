#!/bin/bash


if [ "$#" -ne 1 ]; then
  echo "Need parameter: directory to move log files to"
  exit 0
fi

folder=$1

# change acomms throughput to 100%
sed -i '/DROP_PCT=/c\DROP_PCT=0' launch.sh

#################################################################################
## new scenario: 1gth
#cp 1g_top_half.csv test.csv

#################################################################################
### adp rprop ###################################################################
#################################################################################
## 1 auvs adp
#for idx in `seq 1 30`; do
#	# start sim
#	./launch.sh -a 3 |&tee output.txt &
#	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

#	# wait for sim to run, then end it
#	sleep 3600 && ./kill.sh

#	# move log files to dir
#	sleep 5 && ./mv_logs_to_dir.sh ${folder}/1gth_1auv_adp_np100_random_p$idx

#	# clean up whatever remains
#	sleep 5 && ./clean.sh
#done

#################################################################################
## new scenario: 4g_overlapping
#cp 1g_right_half_low.csv test.csv

#################################################################################
### adp rprop ###################################################################
#################################################################################
## 1 auvs adp
#for idx in `seq 1 30`; do
#	# start sim
#	./launch.sh -a -cg -ng 2 |&tee output.txt &
#	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"

#	# wait for sim to run, then end it
#	sleep 3600 && ./kill.sh

#	# move log files to dir
#	sleep 5 && ./mv_logs_to_dir.sh ${folder}/1grhl_1auv_adp_np100_random_p$idx

#	# clean up whatever remains
#	sleep 5 && ./clean.sh
#done


#################################################################################
## scenario: od
#cp one_depth.csv test.csv

###############################################################################
# 1 auv lm
#for idx in `seq 1 1`; do
	# start sim
#	./launch.sh -a 3 |&tee output.txt &
#    sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"
#    pGP_AUV targ_anna.moos pGP_AUV |&tee pA_output.txt &

	# wait for sim to run, then end it
#	sleep 2500 && ./kill.sh
#    ps ax | grep pGP_AUV | grep -v gedit | awk '{print $1}'| xargs kill -9

	# move log files to dir
#	sleep 5 && ./mv_logs_to_dir.sh ${folder}/od_1auv_b_7_$idx

#    mv pA_output.txt ../../../../mission-eval/od_budget_7.txt

	# clean up whatever remains
#	sleep 5 && ./clean.sh
#done

#################################################################################
## scenario: td
#cp two_depths.csv test.csv
cp b2_5small.csv test.csv

################################################################################
# 1 auv lm

for idx in `seq 5 20`; do
	# start sim
	./launch.sh -a 3 --bigger2 |&tee output.txt &
	sleep 30 && uPokeDB targ_shoreside.moos "DEPLOY_ALL=true"
    pGP_AUV targ_anna.moos pGP_AUV |&tee pA_output.txt &


	# wait for sim to run, then end it
	sleep 8000 && ./kill.sh
    ps ax | grep pGP_AUV | grep -v gedit | awk '{print $1}'| xargs kill -9

	# move log files to dir
	sleep 5 && ./mv_logs_to_dir.sh ${folder}/od_1auv_grd_$idx

    mv pA_output.txt ${folder}/od_1auv_grd_$idx

	# clean up whatever remains
	sleep 5 && ./clean.sh
done
