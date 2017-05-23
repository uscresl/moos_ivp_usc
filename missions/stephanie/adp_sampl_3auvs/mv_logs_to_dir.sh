#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Need parameter: directory to move log files to"
  exit 0
fi

folder=$1

mkdir $1
mv LOG* log* targ* *_predictions*.csv pGP_*.txt hp_optim* output.txt node_* vor_* gp_* stage* state* *.log $folder
