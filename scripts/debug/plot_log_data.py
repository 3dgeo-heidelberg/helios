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
import os


# ---  PREPARE OUTPUT DIRECTORY  --- #
# ---------------------------------- #
if len(sys.argv) < 2:
    print(
        '\nPlot from logs data cannot be generated.\n'
        'It is necessary to specify 2 input arguments:\n'
        '\t1: Path to input data CSV\n'
        '\t2: Path to output directory where plots will be exported\n'
    )
    sys.exit(2)
if not os.path.isdir(sys.argv[2]):
    if os.path.exists(sys.argv[2]):
        print(
            '"{path}" exists but it is not a directory'
            .format(path=sys.argv[2])
        )
        sys.exit(2)
    else:
        os.mkdir(sys.argv[2], 0o755)
outdir = '{path}{sep}'.format(path=sys.argv[2], sep=os.path.sep)


# ---  LOAD DATA  --- #
# ------------------- #
BASE_GROUP_LIST = [
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
def title_from_index(index):
    # Extract subindices
    kdtNum = index[0]
    sahNodes = index[1]
    parallelNum = index[2]
    chunkSize = index[3]
    whFactor = index[4]

    # Text from subindices
    kdtType = 'UNKNOWN'
    parallelType = 'UNKNOWN'
    if kdtNum == 1:
        kdtType = 'Simple'
    elif kdtNum == 2:
        kdtType = 'SAH'
    elif kdtNum == 3:
        kdtType = 'AxisSAH'
    elif kdtNum == 4:
        kdtType = 'FastSAH'
    if parallelNum == 0:
        parallelType = 'Static scheduling with chunk size {chunkSize}'\
            .format(chunkSize=chunkSize)
        if chunkSize < 0:
            parallelType = 'Dynamic scheduling with chunk size {chunkSize}'\
                .format(chunkSize=chunkSize)
    elif parallelNum == 1:
        parallelType = 'Warehouse x{whFactor} with chunk size {chunkSize}'\
            .format(whFactor=whFactor, chunkSize=chunkSize)

    # Return title
    return \
        'Helios with {kdtType} KDTree of {sahNodes} nodes\n'\
        '{parallelType} parallelization'\
        .format(
            kdtType=kdtType,
            sahNodes=sahNodes,
            parallelType=parallelType
        )


def name_from_index(index):
    # Extract subindices
    kdtNum = index[0]
    sahNodes = index[1]
    parallelNum = index[2]
    chunkSize = index[3]
    whFactor = index[4]
    return 'KDT{kdt}_SAH{sah}_PS{ps}_CS_{cs}_WF{wf}'\
        .format(
            kdt=kdtNum,
            sah=sahNodes,
            ps=parallelNum,
            cs=chunkSize,
            wf=whFactor
        )


# Function to configure plots
def configure_plot(
    ax, xlabel='Cores', ylabel='Seconds',
    axx=None, yxlabel='Speed-up'
):
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.autoscale(enable=True, axis='both', tight=True)
    ax.grid('both')
    ax.set_axisbelow(True)
    ax.legend(loc='center left')
    if axx is not None:
        axx.legend(loc='center right')
        axx.set_ylabel(yxlabel)
        axx.autoscale(enable=True, axis='both', tight=True)


for DF in DFS:
    # Extract data for the plot
    KDT = DF.drop('SimulationCores', axis=1).groupby(
        BASE_GROUP_LIST+['KDTBuildingCores']
    ).mean()
    SIM = DF.drop('KDTBuildingCores', axis=1).groupby(
        BASE_GROUP_LIST+['SimulationCores']
    ).mean()

    kdtCores = [x[5] for x in KDT.index]
    kdtTime = [x for x in KDT['KDTBuildTime']]
    kdtTime1 = KDT['KDTBuildTime'].iloc[0]
    kdtSpeedup = [kdtTime1/x for x in KDT['KDTBuildTime']]
    simCores = [x[5] for x in SIM.index]
    simTime = [x for x in SIM['SimulationTime']]
    simTime1 = SIM['SimulationTime'].iloc[0]
    simSpeedup = [simTime1/x for x in SIM['SimulationTime']]
    fullTime = np.add(kdtTime, simTime)
    fullTime1 = fullTime[0]
    fullSpeedup = [fullTime1/x for x in fullTime]

    # Debug section BEGIN ---
    print('DF:\n', DF)
    print('\nKDT:\n', KDT)
    print('\nSIM:\n', SIM)
    # --- Debug section END

    # Prepare plot
    fig = plt.figure(figsize=(16, 9))
    plt.suptitle(title_from_index(KDT.index[0]))

    # Subplot 1
    ax = plt.subplot(2, 2, 1)
    ax.plot(
        kdtCores,
        kdtTime,
        lw=2,
        ls='-',
        color='blue',
        label='KDT building time'
    )
    axx = ax.twinx()
    axx.plot(
        kdtCores,
        kdtCores,
        color='black',
        lw=1,
        ls='--',
        label='Ideal speed-up'
    )
    axx.plot(
        kdtCores,
        kdtSpeedup,
        lw=1,
        ls='-',
        color='black',
        label='KDT building speed-up'
    )
    configure_plot(ax, xlabel='KDT Building cores', axx=axx)

    # Subplot 2
    ax = plt.subplot(2, 2, 2)
    ax.plot(
        simCores,
        simTime,
        lw=2,
        ls='-',
        color='green',
        label='Simulation time'
    )
    axx = ax.twinx()
    axx.plot(
        simCores,
        simCores,
        color='black',
        lw=1,
        ls='--',
        label='Ideal speed-up'
    )
    axx.plot(
        simCores,
        simSpeedup,
        color='black',
        lw=1,
        ls='-',
        label='Simulation speed-up'
    )
    configure_plot(ax, xlabel='Simulation cores', axx=axx)

    # Subplot 3
    ax = plt.subplot(2, 2, 3)
    ax.plot(
        simCores,
        fullTime,
        lw=2,
        ls='-',
        color='red',
        label='Total time'
    )
    axx = ax.twinx()
    axx.plot(
        simCores,
        simCores,
        color='black',
        lw=1,
        ls='--',
        label='Ideal speed-up'
    )
    axx.plot(
        simCores,
        fullSpeedup,
        color='black',
        lw=1,
        ls='-',
        label='Simulation speed-up'
    )
    configure_plot(ax, axx=axx)

    # Subplot 4
    ax = plt.subplot(2, 2, 4)
    ax.plot(
        kdtCores,
        kdtTime,
        lw=2,
        ls='--',
        color='blue',
        label='KDT building time'
    )
    ax.plot(
        simCores,
        simTime,
        lw=2,
        ls='--',
        color='green',
        label='Simulation time'
    )
    ax.plot(
        simCores,
        fullTime,
        lw=2,
        ls='-',
        color='red',
        label='Total time'
    )
    axx = ax.twinx()
    axx.plot(
        simCores,
        simCores,
        color='black',
        lw=1,
        ls='--',
        label='Ideal speed-up'
    )
    axx.plot(
        simCores,
        fullSpeedup,
        color='black',
        lw=1,
        ls='-',
        label='Simulation speed-up'
    )
    configure_plot(ax, axx=axx)

    # Post-process plot
    plt.tight_layout()

    # Export plot
    # plt.show()
    plt.savefig(
        fname='{outpath}'.format(outpath='{outdir}{plotname}'
                                 .format(
                                    outdir=outdir,
                                    plotname=name_from_index(KDT.index[0])
                                 ))
    )
    plt.close()
    plt.cla()
    plt.clf()
    # sys.exit(0)  # TODO Rethink : Comment/uncomment based on debugging
