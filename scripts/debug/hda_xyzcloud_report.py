import hda_diff_report
import pandas as pd
import numpy as np
from scipy.spatial import KDTree as KDT
import matplotlib as mpl
import matplotlib.pyplot as plt


# ---  FUNCTIONS  --- #
# ------------------- #
def print_help():
    print(
'''
Input arguments:
    1 -> Path to first XYZ point cloud file
    2 -> Path to second XYZ point cloud file
'''
    )


def parse_args():
    """Parse input arguments. Raise an exception if not correct arguments were
    given"""
    return hda_diff_report.parse_args(helpf=print_help)


def validate_file_path(path):
    """Check path points to a valid existent file"""
    return hda_diff_report.validate_file_path(path)


def read_data(path):
    """Read data from given file path"""
    return pd.read_csv(path, delimiter=' ', header=None, usecols=[0, 1, 2])\
        .to_numpy()


def compare_data(data, datb, eps=0.001, knn=False):
    """Compare data and datb, returning its differeces"""
    na, nb = len(data), len(datb)
    n = min(na, nb)
    Q, D = None, None  # Neighbors of data (P), and distances
    if not knn:  # Sort comparison
        extract = lambda p, n: (
            np.sort(p[:n, 0]),
            np.sort(p[:n, 1]),
            np.sort(p[:n, 2])
        )
        xa, ya, za = extract(data, n)
        xb, yb, zb = extract(datb, n)
        dx, dy, dz = np.abs(xa-xb), np.abs(ya-yb), np.abs(za-zb)
    else:  # KNN comparison
        bKdt = KDT(datb)
        D, I = bKdt.query(data)
        Q = datb[I]
        dxyz = np.abs(data-Q)
        dx, dy, dz = dxyz[:, 0], dxyz[:, 1], dxyz[:, 2]
        dxyz = None
    return {
        'na': na,
        'nb': nb,
        'dn': abs(na-nb),
        'eps': eps,
        'ndx': np.count_nonzero(dx > eps),
        'ndy': np.count_nonzero(dy > eps),
        'ndz': np.count_nonzero(dz > eps),
        'dxmin': np.min(dx),
        'dymin': np.min(dy),
        'dzmin': np.min(dz),
        'dxmax': np.max(dx),
        'dymax': np.max(dy),
        'dzmax': np.max(dz),
        'dxsum': np.sum(dx),
        'dysum': np.sum(dy),
        'dzsum': np.sum(dz),
        'dxmean': np.mean(dx),
        'dymean': np.mean(dy),
        'dzmean': np.mean(dz),
        'dxstd': np.std(dx),
        'dystd': np.std(dy),
        'dzstd': np.std(dz),
        'Q': Q,  # pi is the ith point in data, qi is its nearneigh in datb
        'D': D  # The distances between P and Q (data and datb neighs)
    }


def pq_subplot(
    px, py, qx, qy, ax=plt,
    title_size=18, label_size=14, tick_size=12, psize=8,
    title=None, xlabel=None, ylabel=None,
):
    """Do a subplot of P and Q as 2D point clouds"""
    ax.set_title(title, fontsize=title_size)
    ax.scatter(px, py, c='blue', s=psize, label='$P$')
    ax.scatter(qx, qy, c='red', s=psize, label='$Q$')
    for i in range(len(px)):
        ax.plot([px[i], qx[i]], [py[i], qy[i]], lw=1, ls='-', color='black')
    ax.grid('both')
    ax.set_axisbelow(True)
    ax.legend(loc='best')
    ax.set_xlabel(xlabel, fontsize=label_size)
    ax.set_ylabel(ylabel, fontsize=label_size)
    ax.tick_params(which='both', labelsize=tick_size)


def report_diff(data, diff, plot=False):
    """Print the output of the compare_data function"""
    print(
'''
The difference in number of points: {dn}
The number of differences > {eps} in (x, y, z): ({ndx}, {ndy}, {ndz})
The minimum difference in (x, y, z): ({dxmin:.5f}, {dymin:.5f}, {dzmin:.5f})
The maximum difference in (x, y, z): ({dxmax:.5f}, {dymax:.5f}, {dzmax:.5f})
The accumulated difference in (x, y, z): ({dxsum:.5f}, {dysum:.5f}, {dzsum:.5f})
The mean difference in (x, y, z): ({dxmean:.5f}, {dymean:.5f}, {dzmean:.5f})
The standard deviation of differences in (x, y, z): ({dxstd:.5f}, {dystd:.5f}, {dzstd:.5f})
'''.format(
    dn=diff['dn'],
    eps=diff['eps'],
    ndx=diff['ndx'],
    ndy=diff['ndy'],
    ndz=diff['ndz'],
    dxmin=diff['dxmin'],
    dymin=diff['dymin'],
    dzmin=diff['dzmin'],
    dxmax=diff['dxmax'],
    dymax=diff['dymax'],
    dzmax=diff['dzmax'],
    dxsum=diff['dxsum'],
    dysum=diff['dysum'],
    dzsum=diff['dzsum'],
    dxmean=diff['dxmean'],
    dymean=diff['dymean'],
    dzmean=diff['dzmean'],
    dxstd=diff['dxstd'],
    dystd=diff['dystd'],
    dzstd=diff['dzstd']
))
    Q = diff.get('Q', None)
    D = diff.get('D', None)
    if Q is not None and plot:
        # Prepare data
        pmin = np.min(data, axis=0)
        P = data - pmin
        Q = Q - pmin
        # Begin plot
        fig = plt.figure(figsize=(18, 12))
        gs = mpl.gridspec.GridSpec(2, 1, height_ratios=[5, 2])
        ugs = gs[0].subgridspec(1, 3)  # Upper subgrid spec
        # Left plot (x, y)
        ax = fig.add_subplot(ugs[0])
        pq_subplot(
            P[:, 0], P[:, 1], Q[:, 0], Q[:, 1], ax=ax,
            title='$(x, y)$', xlabel='$x$', ylabel='$y$'
        )
        # Middle plot (x, y)
        ax = fig.add_subplot(ugs[1])
        pq_subplot(
            P[:, 0], P[:, 2], Q[:, 0], Q[:, 2], ax=ax,
            title='$(x, z)$', xlabel='$x$', ylabel='$z$',
        )
        # Right plot (y, z)
        ax = fig.add_subplot(ugs[2])
        pq_subplot(
            P[:, 1], P[:, 2], Q[:, 1], Q[:, 2], ax=ax,
            title='$(y, z)$', xlabel='$y$', ylabel='$z$'
        )
        # Bottom plot
        if D is not None:
            ax = fig.add_subplot(2, 1, 2)
            ax.hist(D, bins=64)
            ax.grid(visible=True, axis='y')
            ax.set_axisbelow(True)
            ax.set_xlabel('Distance', fontsize=14)
            ax.set_title('Histogram of distances')
        # End plot
        fig.tight_layout()
        # Show plot
        plt.show()


# ---  M A I N  --- #
# ----------------- #
if __name__ == '__main__':
    args = parse_args()
    data, datb = read_data(args['data_path']), read_data(args['datb_path'])
    diff = compare_data(data, datb, knn=True)
    report_diff(data, diff, plot=True)
