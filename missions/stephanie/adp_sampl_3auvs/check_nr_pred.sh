#!/bin/bash

file=$1

wc -l $file/*pred_mean.csv
wc -l $file/*pred_var.csv
wc -l $file/*post_mu.csv
wc -l $file/*post_sigma2.csv

