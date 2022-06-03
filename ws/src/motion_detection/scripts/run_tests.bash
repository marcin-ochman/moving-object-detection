#!/usr/bin/env bash
RED='\033[0;31m'
NC='\033[0m'

for gamma in 0.02 0.05 0.07 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9
do
    for threshold in 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9
    do
        for mu in 0.001 0.005 0.01 0.05 0.1 0.2
        do
            echo -e "${RED} Testing Gamma: $gamma  Threshold: $threshold \n ${NC} Mu: $mu"
            roslaunch motion_detection motion_detection.launch  gamma:=${gamma} threshold:=${threshold} mu:=${mu}  & (sleep 100; rosnode kill --all)
        done
    done
done
