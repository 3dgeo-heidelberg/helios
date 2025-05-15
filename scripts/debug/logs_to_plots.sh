#!/bin/bash

# Author : Alberto M. Esmoris Pena
# Script to extract data from -vt helios logs and then plot it
# Log files must end with following name scheme:
#  *_KDT#_SAH#_PS#_CS#_WF#_SC#_BC#
# Where # stands for a number
# Input arguments:
#  $1 -> Path to logs dir
#  $2 -> Path to output file
#  $3 -> Path to plots output directory
# See : plot_log_data.py


# ---  HANDLE INPUT ARGUMENTS  --- #
# -------------------------------- #
if [[ $# -lt 1 ]]; then
    echo -e '\nPath to logs dir was not provided as first argument\n'
    exit
fi
if [[ ! -d $1 ]]; then
    echo -e '\n"'$1'" is not a directory\n'
    exit
fi
if [[ $# -lt 2 ]]; then
    echo -e '\nPath to output file was not given as second argument\n'
    exit
fi


# --- PREPARE OUTPUT  --- #
# ----------------------- #
outdir=$(sed 's/[^\/]*$//' <<< $2)
mkdir -p $outdir
cat << EOF > $2
KDTreeType,SAHNodes,ParallelizationStrategy,ChunkSize,WarehouseFactor,SimulationCores,KDTBuildingCores,KDTBuildTime,SimulationTime
EOF


# ---  EXTRACT DATA FROM LOGS  --- #
# -------------------------------- #
for logf in $(ls $1'/'*.log); do
    echo 'Digesting "'$logf'" ...'
    logname=$(sed 's/.*\///g' <<< $logf)
    logname=$(sed 's/.*KDT//g' <<< $logname)
    KDT=$(sed 's/_.*//' <<< $logname)
    logname=$(sed 's/.*SAH//g' <<< $logname)
    SAH=$(sed 's/_.*//' <<< $logname)
    logname=$(sed 's/.*PS//g' <<< $logname)
    PS=$(sed 's/_.*//' <<< $logname)
    logname=$(sed 's/.*CS//g' <<< $logname)
    CS=$(sed 's/_.*//' <<< $logname)
    logname=$(sed 's/.*WF//g' <<< $logname)
    WF=$(sed 's/_.*//' <<< $logname)
    logname=$(sed 's/.*SC//g' <<< $logname)
    SC=$(sed 's/_.*//' <<< $logname)
    logname=$(sed 's/.*BC//g' <<< $logname)
    BC=$(sed 's/\..*//' <<< $logname)
    kdt_time=$(grep 'KD built in' "$logf" | sed 's/KD\ built\ in\ //' | sed 's/s//' | tr -d '\r')
    sim_time=$(grep 'Pulse computation tasks finished in' "$logf" | tr -d '\r')
    sim_time=$(sed 's/.*finished\ in\ //' <<< ${sim_time} | sed 's/\ sec.*//')
    cat << EOF >> $2
${KDT},${SAH},${PS},${CS},${WF},${SC},${BC},${kdt_time},${sim_time}
EOF
done


# ---  PLOT DATA  --- #
# ------------------- #
if [[ $# -lt 3 ]]; then
    cat << EOF
Plots will not be automatically generated because no path was given as third
argument. Therefore, it is not known where plots must be stored
EOF
    exit
fi
if ! $(module is-loaded matplotlib/3.3.1-python-3.8.1); then
    module load matplotlib/3.3.1-python-3.8.1
fi
if ! $(module is-loaded pandas/1.0.0-python-3.8.1); then
    module load pandas/1.0.0-python-3.8.1
fi
if ! $(module is-loaded numpy/1.18.1-python-3.8.1); then
    module load numpy/1.18.1-python-3.8.1
fi
python plot_log_data.py "$2" "$3"
