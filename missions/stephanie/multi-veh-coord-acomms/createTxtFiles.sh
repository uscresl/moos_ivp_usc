#!/bin/bash

# script to create txt files for reading into matlab for post-mission analysis
mkdir txtFiles

function extract {
  grep " NAV_X " LOG_${1}*/*.alog | grep uSimMarine | awk '{print $1}' > navxt_${1}.txt
  grep " NAV_X " LOG_${1}*/*.alog | grep uSimMarine | awk '{print $4}' > navx_${1}.txt
  grep " NAV_Y " LOG_${1}*/*.alog | grep uSimMarine | awk '{print $1}' > navyt_${1}.txt
  grep " NAV_Y " LOG_${1}*/*.alog | grep uSimMarine | awk '{print $4}' > navy_${1}.txt

  grep " FORMATION_WAYPOINT_UPDATE " LOG_${1}*/*.alog | grep pFormationWptUpdater | awk '{print $1}' > wptst_${1}.txt
  grep " FORMATION_WAYPOINT_UPDATE " LOG_${1}*/*.alog | grep pFormationWptUpdater | awk '{print $4}' > wpts_${1}.txt

  sed 's/[, =]/ /g' wpts_${1}.txt | awk '{print $2}' > wpts_${1}_x.txt
  sed 's/[, =]/ /g' wpts_${1}.txt | awk '{print $3}' > wpts_${1}_y.txt
}

function extract2 {
  grep "VIEW_POINT " LOG_${1}*/*.alog | grep "bhv_trail" | awk '{print $4}' > lf_wpts_${1}.txt
  grep "VIEW_POINT " LOG_${1}*/*.alog | grep "bhv_trail" | awk '{print $1}' > lf_wptst_${1}.txt
  sed 's/[= ,]/ /g' lf_wpts_${1}.txt | awk '{print $2}' > lf_wpts_${1}_x.txt
  sed 's/[= ,]/ /g' lf_wpts_${1}.txt | awk '{print $4}' > lf_wpts_${1}_y.txt
}

extract A
extract F
extract G

extract2 A
extract2 F
extract2 G

mv nav*.txt txtFiles
mv wpts*.txt txtFiles
mv lf_wpts*.txt txtFiles

