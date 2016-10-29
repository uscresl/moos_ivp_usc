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
}

grep_all LOG_A a
grep_all LOG_B b
