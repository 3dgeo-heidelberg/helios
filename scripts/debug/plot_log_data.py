#!/bin/python3

# Author : Alberto M. Esmoris Pena
# Script to plot performance data from logs
# See : logs_to_plots.sh


# ---  IMPORTS  --- #
# ----------------- #
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys


# ---  LOAD DATA  --- #
# ------------------- #
BASE_GROUP_LIST=[
    'KDTreeType', 'SAHNodes',
    'ParallelizationStrategy', 'ChunkSize', 'WarehouseFactor'
]
DF = pd.read_csv(sys.argv[1], header=0)
GDF = DF.groupby([
    'KDTreeType', 'SAHNodes',
    'ParallelizationStrategy', 'ChunkSize', 'WarehouseFactor',
    'SimulationCores', 'KDTBuildingCores'
]).mean()
GDF = DF.groupby(BASE_GROUP_LIST, as_index=True)
DFS = [GDF.get_group(g) for g in GDF.groups]


# ---  PLOT  --- #
# -------------- #
for DF in DFS:
    KDT = DF.drop('SimulationCores', axis=1).groupby(
        BASE_GROUP_LIST+['KDTBuildingCores']
    ).mean()
    print(KDT)
    SIM = DF.drop('KDTBuildingCores', axis=1).groupby(
        BASE_GROUP_LIST+['SimulationCores']
    ).mean()
    print(SIM)
    plt.figure(figsize=(16, 9))
    ax = plt.subplot(2, 2, 1)
    ax.plot(KDT['KDTBuildingCores'], KDT['KDTBuildTime'])
    plt.savefig()
    sys.exit(0)  # TODO Remove

