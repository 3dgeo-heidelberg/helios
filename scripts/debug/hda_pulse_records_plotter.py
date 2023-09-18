import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt


# ---  FUNCTIONS  --- #
# ------------------- #
def print_help():
    print(
'''
Input arguments:
    1 -> Path to the first directory containing simulation records
    2 -> Path to the second directory containing simulation records
    3 -> Path to the directory where plots will be stored
'''
    )


def parse_args(helpf=print_help):
    """Parse input arguments. Raise an exception if not correct arguments were
    given"""
    if len(sys.argv) == 1:
        helpf()
        exit(0)
    elif len(sys.argv) < 4:
        raise Exception(
            '{m} arguments were given but 3 are required'
            .format(m=len(sys.argv)-1)
        )
    dira_path = sys.argv[1]
    if not validate_directory(dira_path):
        raise Exception(
            'The directory "{d}"\n'
            'was given as the first directory of records, but it is not valid'
        )
    dirb_path = sys.argv[2]
    if not validate_directory(dirb_path):
        raise Exception(
            'The directory "{d}"\n'
            'was given as the second directory of records, but it is not valid'
        )
    dirout_path = sys.argv[3]
    if not validate_directory(dirout_path):
        raise Exception(
            'The directory "{d}"\n'
            'was given as the third directory for plots, but it is not valid'
        )
    return {
        'dira_path': dira_path,
        'dirb_path': dirb_path,
        'dirout_path': dirout_path
    }


def validate_directory(path):
    """Check path points to a valid existent directory.
    If it does not exist and it cannot be created, then it is not valid"""
    if os.path.exists(path):
        if os.path.isdir(path):
            return True
        else:
            return False
    else:
        os.mkdir(path, mode=0o775)
        if os.path.exists(path):
            return True
    return False


def read_records(path, sep=','):
    """Read all record files contained in the directory pointed by given
    path"""
    # Read vectorial records
    intensity_calc = read_record(os.path.join(
        path, 'intensity_calc.csv'
    ), sep)
    # Return key-word records
    return {
        # Intensity calculation records
        'incidence_angle_rad': intensity_calc[:, 0],
        'target_range_m': intensity_calc[:, 1],
        'target_area_m2': intensity_calc[:, 2],
        'radius_m': intensity_calc[:, 3],
        'bdrf': intensity_calc[:, 4],
        'cross_section': intensity_calc[:, 5],
        'received_power': intensity_calc[:, 6]
    }


def read_record(path, sep):
    """Read given record file
    :param path: The path to the record file to be read
    :param sep: The separator used in the record file
    :return: None if the record file could not be read, the data as a numpy
        array otherwise
    """
    if os.path.exists(path) and os.path.isfile(path):
        return np.loadtxt(path, delimiter=sep)
    return None


def plot_records(arec, brec, outdir):
    """Do plots for each case and export them in given output directory
    :param arec: The records of the first case
    :param brec: The records of the second case
    :param outdir: The output directory where generated plots must be
        written
    """
    do_incidence_angle_plots(arec, brec, outdir)
    do_by_incidence_angle_plots(arec, brec, outdir)


def validate_record(key, rec, recid):
    """Check that the record with given key is available
    :return: True if the record is valid (available data), False otherwise
    """
    if key not in rec or rec.get(key, None) is None:
        print(
            'Record "{key}" is not available for {recid} records'
            .format(
                key=key,
                recid=recid
            )
        )
        return False
    return True


def init_figure(figsize=(20, 12)):
    """Initialize a matplotlib's figure context"""
    fig = plt.figure(
        figsize=figsize
    )
    return fig


def do_incidence_angle_subplot(
    fig, ax, phi, label=None, title=None, xlabel=None, ylabel=None, bins=32,
    log=False
):
    if title is not None:
        ax.set_title(title, fontsize=20)
    hist = ax.hist(phi, bins=bins, label=label, log=log)
    ax.axvline(x=np.mean(phi), color='tab:orange', lw=3, label='$\\mu$')
    if xlabel is not None:
        ax.set_xlabel(xlabel, fontsize=16)
    if ylabel is not None:
        ax.set_ylabel(ylabel, fontsize=16)
    ax.tick_params(axis='both', which='both', labelsize=14)
    ax.legend(loc='upper right', fontsize=14)
    ax.grid('both')
    ax.set_axisbelow(True)


def do_incidence_angle_plots(arec, brec, outdir):
    # Validate incidence angle data
    if not validate_record('incidence_angle_rad', arec, 'a') or \
            not validate_record('incidence_angle_rad', brec, 'b'):
        print('Cannot do incidence angle plots')
        return

    # Do the incidence angle plots
    fig = init_figure()  # Initialize figure
    ax = fig.add_subplot(2, 2, 1)  # Initialize phi(a) subplot
    do_incidence_angle_subplot(
        fig, ax, arec['incidence_angle_rad'],
        label='$\\varphi(a)$',
        title='A-Incidence angle ($\\varphi$) in rad',
        xlabel='$\\varphi(a)$',
        ylabel='cases'
    )
    ax = fig.add_subplot(2, 2, 3)  # Initialize phi(a) log subplot
    do_incidence_angle_subplot(
        fig, ax, arec['incidence_angle_rad'],
        label='$\\varphi(a)$',
        title='A-Incidence angle ($\\varphi$) in rad (logarithmic)',
        xlabel='$\\varphi(a)$',
        ylabel='cases',
        log=True
    )
    ax = fig.add_subplot(2, 2, 2)  # Initialize phi(b) subplot
    do_incidence_angle_subplot(
        fig, ax, brec['incidence_angle_rad'],
        label='$\\varphi(b)$',
        title='B-Incidence angle ($\\varphi$) in rad',
        xlabel='$\\varphi(b)$',
        ylabel='cases'
    )
    ax = fig.add_subplot(2, 2, 4)  # Initialize phi(a) log subplot
    do_incidence_angle_subplot(
        fig, ax, brec['incidence_angle_rad'],
        label='$\\varphi(b)$',
        title='B-Incidence angle ($\\varphi$) in rad (logarithmic)',
        xlabel='$\\varphi(b)$',
        ylabel='cases',
        log=True
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(
        os.path.join(outdir, 'incidence_angle_distribution.png')
    )
    fig.clear()
    plt.close(fig)


def do_y_by_x_subplot(
    fig, ax, x, y, title=None, xlabel=None, ylabel=None,
    color='black'
):
    if title is not None:
        ax.set_title(title, fontsize=14)
    ax.scatter(x, y, c=color, s=8)
    if xlabel is not None:
        ax.set_xlabel(xlabel, fontsize=12)
    if ylabel is not None:
        ax.set_ylabel(ylabel, fontsize=12)
    ax.tick_params(axis='both', which='both', labelsize=12)
    ax.grid('both')
    ax.set_axisbelow(True)


def do_by_incidence_angle_plots(arec, brec, outdir):
    # Validate classification calculation data
    if not validate_record('incidence_angle_rad', arec, 'a') or \
            not validate_record('target_range_m', arec, 'a') or \
            not validate_record('target_area_m2', arec, 'a') or \
            not validate_record('radius_m', arec, 'a') or \
            not validate_record('bdrf', arec, 'a') or \
            not validate_record('cross_section', arec, 'a') or \
            not validate_record('received_power', arec, 'a') or \
            not validate_record('incidence_angle_rad', brec, 'b') or \
            not validate_record('target_range_m', brec, 'b') or \
            not validate_record('target_area_m2', brec, 'b') or \
            not validate_record('radius_m', brec, 'b') or \
            not validate_record('bdrf', brec, 'b') or \
            not validate_record('cross_section', brec, 'b') or \
            not validate_record('received_power', brec, 'b'):
        print('Cannot do by incidence angle plots')
        return
    # Do the "by incidence angle" plots
    fig = init_figure()  # Initialize figure
    ax = fig.add_subplot(3, 4, 1)  # Initialize target range A subplot
    do_y_by_x_subplot(
        fig, ax, arec['incidence_angle_rad'], arec['target_range_m'],
        title='A-Target range (m)',
        xlabel='Incidence angle (rad)',
        ylabel='Target range (m)',
        color='black'
    )
    ax = fig.add_subplot(3, 4, 2)  # Initialize target area A subplot
    do_y_by_x_subplot(
        fig, ax, arec['incidence_angle_rad'], arec['target_area_m2'],
        title='A-Target area ($m^2$)',
        xlabel='Incidence angle (rad)',
        ylabel='Target area ($m^2$)',
        color='tab:blue'
    )
    ax = fig.add_subplot(3, 4, 5)  # Initialize radius A subplot
    do_y_by_x_subplot(
        fig, ax, arec['incidence_angle_rad'], arec['radius_m'],
        title='A-Radius (m)',
        xlabel='Incidence angle (rad)',
        ylabel='Radius (m)',
        color='tab:red'
    )
    ax = fig.add_subplot(3, 4, 6)  # Initialize BDRF A subplot
    do_y_by_x_subplot(
        fig, ax, arec['incidence_angle_rad'], arec['bdrf'],
        title='A-BDRF',
        xlabel='Incidence angle (rad)',
        ylabel='BDRF',
        color='tab:green'
    )
    ax = fig.add_subplot(3, 4, 9)  # Initialize Cross-section A subplot
    do_y_by_x_subplot(
        fig, ax, arec['incidence_angle_rad'], arec['cross_section'],
        title='A-Cross-section ($m^2$)',
        xlabel='Incidence angle (rad)',
        ylabel='Cross-section ($m^2$)',
        color='tab:orange'
    )
    ax = fig.add_subplot(3, 4, 10)  # Initialize received power A subplot
    do_y_by_x_subplot(
        fig, ax, arec['incidence_angle_rad'], arec['received_power'],
        title='A-Received power',
        xlabel='Incidence angle (rad)',
        ylabel='Received power',
        color='tab:purple'
    )
    ax = fig.add_subplot(3, 4, 3)  # Initialize target range B subplot
    do_y_by_x_subplot(
        fig, ax, brec['incidence_angle_rad'], brec['target_range_m'],
        title='B-Target range (m)',
        xlabel='Incidence angle (rad)',
        ylabel='Target range (m)',
        color='black'
    )
    ax = fig.add_subplot(3, 4, 4)  # Initialize target area B subplot
    do_y_by_x_subplot(
        fig, ax, brec['incidence_angle_rad'], brec['target_area_m2'],
        title='B-Target area ($m^2$)',
        xlabel='Incidence angle (rad)',
        ylabel='Target area ($m^2$)',
        color='tab:blue'
    )
    ax = fig.add_subplot(3, 4, 7)  # Initialize radius B subplot
    do_y_by_x_subplot(
        fig, ax, brec['incidence_angle_rad'], brec['radius_m'],
        title='B-Radius (m)',
        xlabel='Incidence angle (rad)',
        ylabel='Radius (m)',
        color='tab:red'
    )
    ax = fig.add_subplot(3, 4, 8)  # Initialize BDRF B subplot
    do_y_by_x_subplot(
        fig, ax, brec['incidence_angle_rad'], brec['bdrf'],
        title='B-BDRF',
        xlabel='Incidence angle (rad)',
        ylabel='BDRF',
        color='tab:green'
    )
    ax = fig.add_subplot(3, 4, 11)  # Initialize Cross-section B subplot
    do_y_by_x_subplot(
        fig, ax, brec['incidence_angle_rad'], brec['cross_section'],
        title='B-Cross-section ($m^2$)',
        xlabel='Incidence angle (rad)',
        ylabel='Cross-section ($m^2$)',
        color='tab:orange'
    )
    ax = fig.add_subplot(3, 4, 12)  # Initialize received power B subplot
    do_y_by_x_subplot(
        fig, ax, brec['incidence_angle_rad'], brec['received_power'],
        title='B-Received power',
        xlabel='Incidence angle (rad)',
        ylabel='Received power',
        color='tab:purple'
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(
        os.path.join(outdir, 'plots_by_incidence_angle.png')
    )
    fig.clear()
    plt.close(fig)



# ---   M A I N   --- #
# ------------------- #
if __name__ == '__main__':
    # Prepare plotter
    args = parse_args()
    sep = ','
    # Read A records
    print(
        'Reading A-records from "{path}" ...'
        .format(path=args['dira_path'])
    )
    start = time.perf_counter()
    arec = read_records(args['dira_path'], sep=sep)
    end = time.perf_counter()
    print('Read A-records in {t} seconds'.format(t=end-start))
    # Read B records
    print(
        'Reading B-records from "{path}" ...'
        .format(path=args['dirb_path'])
    )
    start = time.perf_counter()
    brec = read_records(args['dirb_path'], sep=sep)
    end = time.perf_counter()
    print('Read B-records in {t} seconds'.format(t=end-start))
    # Plot records
    print(
        'Generating plots at "{path}" ...'
        .format(
            path=args['dirout_path']
        )
    )
    start = time.perf_counter()
    plot_records(arec, brec, args['dirout_path'])
    end = time.perf_counter()
    print('Generated plots in {t} seconds'.format(t=end-start))
