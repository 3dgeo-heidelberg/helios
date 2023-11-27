import sys
import pandas as pd
import csv
import numpy as np
import time
import matplotlib.pyplot as plt


# ---  FUNCTIONS  --- #
# ------------------- #
def print_help():
    print(
        'HELP:\n\n'
        'First argument: Path to fullwave file\n'
        'Second argument [OPTIONAL]: Index of record to be plot\n'
    )


def timef(f, fargs={}, name='UNSPECIFIED FUNCTION'):
    start = time.perf_counter()
    fout = f(**fargs)
    end = time.perf_counter()
    print(
        '{name} run in {time:.3f} seconds'
        .format(
            name=name,
            time=end-start
        )
    )
    return fout


def read_inargs():
    return {
        'help': True if (len(sys.argv) < 2 or
                         sys.argv[1] == '-h' or
                         sys.argv[1] == '--help')
        else False,
        'inpath': sys.argv[1] if len(sys.argv) > 1 else None,
        'fwidx': int(sys.argv[2]) if len(sys.argv) > 2 else None
    }


def load_fullwave(inargs):
    # Get csv rows (pandas not used due to varying number of columns per row)
    rows = []
    with open(inargs['inpath'], 'r') as fwfin:
        csv_reader = csv.reader(fwfin, delimiter=' ')
        for row in csv_reader:
            rows.append(row)
    return rows


def plot_fullwave(fw):
    # Prepare plot
    fig = plt.figure(figsize=(14, 9))
    ax = fig.add_subplot(1, 1, 1)
    # Plot data
    if inargs['fwidx'] is None:
        resolution = 256
        fig.suptitle(f'Fullwaves ({resolution})')
        step = max(1, len(fw)//resolution)
        for fwi in fw[::step]:
            _plot_fullwave(fwi, ax, alpha=1.0, lw=1)
    else:
        fig.suptitle('Fullwave')
        _plot_fullwave(fw[inargs['fwidx']], ax, lw=2)
    # Post-process plot
    ax.grid('both')
    ax.set_xlabel('Time  [$\\mathrm{ns}$]')
    ax.set_axisbelow(True)
    fig.tight_layout()
    # Show plot
    plt.show()


def _plot_fullwave(fw, ax, alpha=1, lw=2):
    tmin, tmax = float(fw[7]), float(fw[8])
    y = np.array(fw[10:], dtype=float)
    t = np.linspace(tmin, tmax, y.shape[0])
    ax.plot(t, y, lw=lw, alpha=alpha)


# ---  M A I N  --- #
# ----------------- #
if __name__ == '__main__':
    inargs = timef(read_inargs, name='Read input arguments')
    if inargs['help']:
        print_help()
        sys.exit(0)
    fw = timef(load_fullwave, fargs={'inargs': inargs}, name='Load fullwave')
    plot_fullwave(fw)
