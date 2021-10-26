#!/bin/bash

#SBATCH -N 1
#SBATCH -n 1
#SBATCH -c 24
#SBATCH -p thinnodes
#SBATCH -t 72:00:00
#SBATCH --mem 120GB
#SBATCH --mail-type=begin
#SBATCH --mail-type=end
#SBATCH --mail-user=albertoesmp@gmail.com

#Author: Alberto M. Esmoris Pena
#
# Run Helios++ in CESGA for benchmarking

# ---  PREPARE ENVIRONMENT  --- #
# ----------------------------- #
# Load environment if it has not been loaded yet
if [[ ${HELIOS_ENV} != 'ON' ]]; then
    source "$HOME/git/helios/scripts/cesga_env.sh"
fi


# ---  SEQUENCE CONFIGURATION  --- #
# --------------------------------- #
# WORKING DIRECTORY
cd "${STORE2}/helios/"
# FIRST THE M SURVEYS, ASSETS, OUTPUTS and LOGNAMES
SURVEY=(
    'data/surveys/als_scenario_fixed_250_B_BR05_1_wS1_5.xml'
    '/mnt/netapp2/Store_uscciaep/helios/data/surveys/als_scenario_scaled_cog_B_BR05_1_wS1_5.xml'
)
ASSET=(
    'assets/'
    'assets/'
)
OUTPUT=(
    'output/'
    'output/'
)
LOGNAME=('log/LOG_FIXED250')
LOGNAME=('log/LOG_SCALEDCOG')
# SECOND THE KDTree-Types and SurfeAreaHeuristic-Nodes
KDTTYPE=(1 2 3 4)
SAHNODES=(0 21 21 32)
# THIRD THE PARALLELIZATION STRATEGY
PARALLELIZATION=(
    0 0 0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0 0
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
)
CHUNK_SIZE=(
    16 16 16 16 16 16 16 16 16 16 16 16
    16 16 16 16 16 16 16 16 16 16 16 16
    32 32 32 32 32 32 32 32 32 32 32 32
    32 32 32 32 32 32 32 32 32 32 32 32
    64 64 64 64 64 64 64 64 64 64 64 64
    64 64 64 64 64 64 64 64 64 64 64 64
    -16 -16 -16 -16 -16 -16 -16 -16 -16 -16 -16 -16
    -16 -16 -16 -16 -16 -16 -16 -16 -16 -16 -16 -16
    -32 -32 -32 -32 -32 -32 -32 -32 -32 -32 -32 -32
    -32 -32 -32 -32 -32 -32 -32 -32 -32 -32 -32 -32
    -64 -64 -64 -64 -64 -64 -64 -64 -64 -64 -64 -64
    -64 -64 -64 -64 -64 -64 -64 -64 -64 -64 -64 -64
    16 16 16 16 16 16 16 16 16 16 16 16
    16 16 16 16 16 16 16 16 16 16 16 16
    16 16 16 16 16 16 16 16 16 16 16 16
    32 32 32 32 32 32 32 32 32 32 32 32
    32 32 32 32 32 32 32 32 32 32 32 32
    32 32 32 32 32 32 32 32 32 32 32 32
    64 64 64 64 64 64 64 64 64 64 64 64
    64 64 64 64 64 64 64 64 64 64 64 64
    64 64 64 64 64 64 64 64 64 64 64 64
)
WAREHOUSE_FACTOR=(
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    1 1 1 1 1 1 1 1 1 1 1 1
    3 3 3 3 3 3 3 3 3 3 3 3
    6 6 6 6 6 6 6 6 6 6 6 6
    9 9 9 9 9 9 9 9 9 9 9 9
    3 3 3 3 3 3 3 3 3 3 3 3
    6 6 6 6 6 6 6 6 6 6 6 6
    9 9 9 9 9 9 9 9 9 9 9 9
    3 3 3 3 3 3 3 3 3 3 3 3
    6 6 6 6 6 6 6 6 6 6 6 6
    9 9 9 9 9 9 9 9 9 9 9 9
)


# FOURTH THE SIMULATION JOBS AND KDT-BUILDING JOBS
SIMCORES=(1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24)
KDTCORES=(1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24)


# ---  SEQUENCE TASKS  --- #
# ------------------------ #
for (( i=0 ; i < ${#SURVEY[@]} ; ++i )); do
    for (( j=0 ; j < ${#KDTTYPE[@]} ; ++j )); do
        for (( p=0 ; p < ${#PARALLELIZATION[@]} ; ++p )); do
            for (( k=0 ; k < ${#SIMCORES[@]} ; ++k )); do
                for (( l=0 ; l < ${#KDTCORES[@]} ; ++l )); do
                    logname="$PWD/${LOGNAME[$i]}_KDT${KDTTYPE[$j]}_SAH${SAHNODES[$j]}_"
                    logname="${logname}PS${PARALLELIZATION[$p]}_CS${CHUNK_SIZE[$p]}_WF${WAREHOUSE_FACTOR[$p]}_"
                    logname="${logname}SC${SIMCORES[$k]}_BC${KDTCORES[$l]}.log"
                    echo 'LOGGING ('"$i,$j,$p,$k,$l"') to "'"${logname}"'"'
                    srun ${HELIOS_BIN} ${SURVEY[$i]} ${ASSET[$i]} ${OUTPUT[$i]} \
                        --rebuildScene --kdt ${KDTTYPE[$j]} --sahNodes ${SAHNODES[$j]} \
                        --parallelization ${PARALLELIZATION[$p]} --chunkSize ${CHUNK_SIZE[$p]} \
                        --warehouseFactor ${WAREHOUSE_FACTOR[$p]} \
                        -j ${SIMCORES[$k]} --kdtJobs ${KDTCORES[$l]} -vt 2>&1> $logname
                    echo -e "\n\n"
                done
            done
        done
    done
done
