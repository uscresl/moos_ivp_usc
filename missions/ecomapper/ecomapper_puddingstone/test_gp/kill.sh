#!/bin/bash
ps ax | grep moos | grep -v gedit | awk '{print $1}' | xargs kill -9

