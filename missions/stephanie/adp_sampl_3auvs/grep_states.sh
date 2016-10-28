#!/bin/bash

grep_all ()
{
  if [ -z "$1" ]
  then
    echo "error: parameter length 0, exiting"
    exit  0
  fi

  grep " STAGE " ${1}*/*.alog > stage_${2}.txt
  grep " STATE_MISSION " ${1}*/*.alog > stateM_${2}.txt
  grep " STATE_SURFACE " ${1}*/*.alog > stateS_${2}.txt
  grep " STATE_DATA_SHARING " ${1}*/*.alog > stateD_${2}.txt
}

grep_all LOG_A a
grep_all LOG_B b
