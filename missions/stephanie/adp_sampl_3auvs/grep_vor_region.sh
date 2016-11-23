#!/bin/bash

grep " VORONOI_REGION " LOG_A*/*.alog > vor_a.txt
grep " VORONOI_REGION " LOG_B*/*.alog > vor_b.txt
grep " VORONOI_REGION " LOG_C*/*.alog > vor_c.txt

