#!/usr/bin/env bash
RED='\033[0;31m'
NC='\033[0m'

for gamma in 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9
do
    for threshold in 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9
    do
        echo -e "${RED} Testing Gamma: $gamma  Threshold: $threshold \n ${NC}"
        roslaunch motion_detection motion_detection.launch  gamma:=${gamma} threshold:=${threshold}  & (sleep 100; rosnode kill --all)
    done
done
