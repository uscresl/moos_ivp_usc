#!/bin/bash

grep " NODE_REPORT " LOG_A*/*.alog > node_a.txt
grep " NODE_REPORT " LOG_B*/*.alog > node_b.txt
grep " NODE_REPORT " LOG_C*/*.alog > node_c.txt

