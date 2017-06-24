#!/bin/bash

file=log_a*.log
if [ -f ${file} ]; then
grep "pGP_AUV :: " log_a*.log > gp_a.txt
fi

file=log_b*.log
if [ -f ${file} ]; then
grep "pGP_AUV :: " log_b*.log > gp_b.txt
fi

file=log_c*.log
if [ -f ${file} ]; then
grep "pGP_AUV :: " log_c*.log > gp_c.txt
fi
