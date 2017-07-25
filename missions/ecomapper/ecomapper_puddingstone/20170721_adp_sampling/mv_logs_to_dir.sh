#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Need parameter: directory to move log files to"
  exit 0
fi

folder=$1

mkdir $1
mv MOOSLog* LOG_* log* predictions*.csv  hp_optim* *.log *.moos ecomapper.bhv LOG_ZOOMER* output.txt $folder
