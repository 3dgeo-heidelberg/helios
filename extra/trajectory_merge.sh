#!/bin/bash


MYDIR=$1'/'
# LEG_IDS=("000" "002" "004" "005" "007" "009")
# LEG_IDS=("000" "001" "002" "003" "004" "005" "006" "007" "008" "009")
LEG_IDS=("000" "001" "002" "003" "004" "005" "006" "007" "008" "009" "010")

echo 'Merging trajectories at "'${MYDIR}'" ...'
rm -f "${MYDIR}merged_trajectory.txt"; for lid in ${LEG_IDS[@]}; do cat "${MYDIR}leg${lid}_trajectory.txt" >> "${MYDIR}merged_trajectory.txt"; done
