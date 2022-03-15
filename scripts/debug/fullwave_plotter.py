import sys
import pandas as pd
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
        'fwidx': int(sys.argv[2]) if len(sys.argv) > 2 else 0
    }


def load_fullwave(inargs):
    return pd.read_csv(
        inargs['inpath'],
        sep=' ',
        header=None,
        skiprows=inargs['fwidx'],
        nrows=1
    ).to_numpy()


def plot_fullwave(fw):
    y = fw[10:]
    t = np.linspace(0, 1, y.shape[0])
    fig = plt.figure(figsize=(14, 9))
    fig.suptitle('Fullwave')
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(t, y, color='red', lw=2)
    ax.grid('both')
    ax.set_axisbelow(True)
    plt.tight_layout()
    plt.show()


# ---  M A I N  --- #
# ----------------- #
if __name__ == '__main__':
    inargs = timef(read_inargs, name='Read input arguments')
    if inargs['help']:
        print_help()
        sys.exit(0)
    fw = timef(load_fullwave, fargs={'inargs': inargs}, name='Load fullwave')
    plot_fullwave(fw[0])
