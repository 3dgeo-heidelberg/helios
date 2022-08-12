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
            'was given as the third directory of records, but it is not valid'
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
    return {
        # Platform position records
        'platform_x': read_record(os.path.join(
            path, 'platform_position_x.csv'
        ), sep),
        'platform_y': read_record(os.path.join(
            path, 'platform_position_y.csv'
        ), sep),
        'platform_z': read_record(os.path.join(
            path, 'platform_position_z.csv'
        ), sep),
        # Platform attitude records
        'platform_roll': read_record(os.path.join(
            path, 'platform_roll.csv'
        ), sep),
        'platform_pitch': read_record(os.path.join(
            path, 'platform_pitch.csv'
        ), sep),
        'platform_yaw': read_record(os.path.join(
            path, 'platform_yaw.csv'
        ), sep),
        # Scanner position records
        'scanner_x': read_record(os.path.join(
            path, 'scanner_position_x.csv'
        ), sep),
        'scanner_y': read_record(os.path.join(
            path, 'scanner_position_y.csv'
        ), sep),
        'scanner_z': read_record(os.path.join(
            path, 'scanner_position_z.csv'
        ), sep),
        # Scanner attitude records
        'scanner_roll': read_record(os.path.join(
            path, 'scanner_roll.csv'
        ), sep),
        'scanner_pitch': read_record(os.path.join(
            path, 'scanner_pitch.csv'
        ), sep),
        'scanner_yaw': read_record(os.path.join(
            path, 'scanner_yaw.csv'
        ), sep),
        # Scanner head attitude records
        'scanner_head_roll': read_record(os.path.join(
            path, 'scanner_head_roll.csv'
        ), sep),
        'scanner_head_pitch': read_record(os.path.join(
            path, 'scanner_head_pitch.csv'
        ), sep),
        'scanner_head_yaw': read_record(os.path.join(
            path, 'scanner_head_yaw.csv'
        ), sep),
        # Beam deflector attitude records
        'deflector_emitting_roll': read_record(os.path.join(
            path, 'deflector_emitting_roll.csv'
        ), sep),
        'deflector_emitting_pitch': read_record(os.path.join(
            path, 'deflector_emitting_pitch.csv'
        ), sep),
        'deflector_emitting_yaw': read_record(os.path.join(
            path, 'deflector_emitting_yaw.csv'
        ), sep),
        # Beam origin records
        'beam_origin_x': read_record(os.path.join(
            path, 'beam_origin_x.csv'
        ), sep),
        'beam_origin_y': read_record(os.path.join(
            path, 'beam_origin_y.csv'
        ), sep),
        'beam_origin_z': read_record(os.path.join(
            path, 'beam_origin_z.csv'
        ), sep),
        # Beam attitude records
        'beam_roll': read_record(os.path.join(
            path, 'beam_roll.csv'
        ), sep),
        'beam_pitch': read_record(os.path.join(
            path, 'beam_pitch.csv'
        ), sep),
        'beam_yaw': read_record(os.path.join(
            path, 'beam_yaw.csv'
        ), sep)
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
    do_platform_position_plots(arec, brec, outdir)
    do_platform_attitude_plots(arec, brec, outdir)
    do_scanner_position_plots(arec, brec, outdir)
    do_scanner_attitude_plots(arec, brec, outdir)
    do_scanner_head_attitude_plots(arec, brec, outdir)
    do_deflector_emitting_attitude_plots(arec, brec, outdir)
    do_beam_origin_plots(arec, brec, outdir)
    do_beam_attitude_plots(arec, brec, outdir)


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


def do_position_subplot(
    fig, ax, x, label=None, title=None, ylabel=None, maxpoints=65536
):
    nsteps = len(x)
    steps = np.arange(1, nsteps+1)
    h = max(1, nsteps//maxpoints)  # Step size
    x, steps = x[::h], steps[::h]
    if title is not None:
        ax.set_title(title, fontsize=20)
    ax.plot(steps, x, label=label, lw=2, color='black')
    ax.set_xlabel('Simulation steps', fontsize=16)
    if ylabel is not None:
        ax.set_ylabel(ylabel, fontsize=16)
    ax.tick_params(axis='both', which='major', labelsize=14)
    ax.tick_params(axis='both', which='minor', labelsize=14)
    ax.legend(loc='upper right', fontsize=14)
    ax.grid('both')
    ax.set_axisbelow(True)


def do_platform_position_plots(arec, brec, outdir):
    # Validate platform position data
    if not validate_record('platform_x', arec, 'a') or \
            not validate_record('platform_y', arec, 'a') or \
            not validate_record('platform_z', arec, 'a') or \
            not validate_record('platform_x', brec, 'b') or \
            not validate_record('platform_y', brec, 'b') or \
            not validate_record('platform_z', brec, 'b'):
        print('Cannot do platform position plots')
        return

    # Do the platform position plots
    fig = init_figure()  # Initialize figure
    ax = fig.add_subplot(2, 3, 1)  # Initialize x(a) subplot
    do_position_subplot(
        fig, ax, arec['platform_x'],
        label='$x(a)$',
        title='A-Platform\'s $x$ position',
        ylabel='$x(a)$'
    )
    ax = fig.add_subplot(2, 3, 2)  # Initialize y(a) subplot
    do_position_subplot(
        fig, ax, arec['platform_y'],
        label='$y(a)$',
        title='A-Platform\'s $y$ position',
        ylabel='$y(a)$'
    )
    ax = fig.add_subplot(2, 3, 3)  # Initialize z(a) subplot
    do_position_subplot(
        fig, ax, arec['platform_z'],
        label='$z(a)$',
        title='A-Platform\'s $z$ position',
        ylabel='$z(a)$'
    )
    ax = fig.add_subplot(2, 3, 4)  # Initialize x(b) subplot
    do_position_subplot(
        fig, ax, brec['platform_x'],
        label='$x(b)$',
        title='B-Platform\'s $x$ position',
        ylabel='$x(b)$'
    )
    ax = fig.add_subplot(2, 3, 5)  # Initialize y(b) subplot
    do_position_subplot(
        fig, ax, brec['platform_y'],
        label='$y(b)$',
        title='B-Platform\'s $y$ position',
        ylabel='$y(b)$'
    )
    ax = fig.add_subplot(2, 3, 6)  # Initialize z(b) subplot
    do_position_subplot(
        fig, ax, brec['platform_z'],
        label='$z(b)$',
        title='B-Platform\'s $z$ position',
        ylabel='$z(b)$'
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(
        os.path.join(outdir, 'platform_position.png')
    )
    fig.clear()
    plt.close(fig)


def do_scanner_position_plots(arec, brec, outdir):
    # Validate scanner position data
    if not validate_record('scanner_x', arec, 'a') or \
            not validate_record('scanner_y', arec, 'a') or \
            not validate_record('scanner_z', arec, 'a') or \
            not validate_record('scanner_x', brec, 'b') or \
            not validate_record('scanner_y', brec, 'b') or \
            not validate_record('scanner_z', brec, 'b'):
        print('Cannot do scanner position plots')
        return

    # Do the scanner position plots
    fig = init_figure()  # Initialize figure
    ax = fig.add_subplot(2, 3, 1)  # Initialize x(a) subplot
    do_position_subplot(
        fig, ax, arec['scanner_x'],
        label='$x(a)$',
        title='A-Scanner\'s $x$ position',
        ylabel='$x(a)$'
    )
    ax = fig.add_subplot(2, 3, 2)  # Initialize y(a) subplot
    do_position_subplot(
        fig, ax, arec['scanner_y'],
        label='$y(a)$',
        title='A-Scanner\'s $y$ position',
        ylabel='$y(a)$'
    )
    ax = fig.add_subplot(2, 3, 3)  # Initialize z(a) subplot
    do_position_subplot(
        fig, ax, arec['scanner_z'],
        label='$z(a)$',
        title='A-Scanner\'s $z$ position',
        ylabel='$z(a)$'
    )
    ax = fig.add_subplot(2, 3, 4)  # Initialize x(b) subplot
    do_position_subplot(
        fig, ax, brec['scanner_x'],
        label='$x(b)$',
        title='B-Scanner\'s $x$ position',
        ylabel='$x(b)$'
    )
    ax = fig.add_subplot(2, 3, 5)  # Initialize y(b) subplot
    do_position_subplot(
        fig, ax, brec['scanner_y'],
        label='$y(b)$',
        title='B-Scanner\'s $y$ position',
        ylabel='$y(b)$'
    )
    ax = fig.add_subplot(2, 3, 6)  # Initialize z(b) subplot
    do_position_subplot(
        fig, ax, brec['scanner_z'],
        label='$z(b)$',
        title='B-Scanner\'s $z$ position',
        ylabel='$z(b)$'
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(
        os.path.join(outdir, 'scanner_position.png')
    )
    fig.clear()
    plt.close(fig)


def do_beam_origin_plots(arec, brec, outdir):
    # Validate beam origin data
    if not validate_record('beam_origin_x', arec, 'a') or \
            not validate_record('beam_origin_y', arec, 'a') or \
            not validate_record('beam_origin_z', arec, 'a') or \
            not validate_record('beam_origin_x', brec, 'b') or \
            not validate_record('beam_origin_y', brec, 'b') or \
            not validate_record('beam_origin_z', brec, 'b'):
        print('Cannot do beam origin plots')
        return

    # Do the beam origin plots
    fig = init_figure()  # Initialize figure
    ax = fig.add_subplot(2, 3, 1)  # Initialize x(a) subplot
    do_position_subplot(
        fig, ax, arec['beam_origin_x'],
        label='$x(a)$',
        title='A-Beam\'s origin $x$ position',
        ylabel='$x(a)$'
    )
    ax = fig.add_subplot(2, 3, 2)  # Initialize y(a) subplot
    do_position_subplot(
        fig, ax, arec['beam_origin_y'],
        label='$y(a)$',
        title='A-Beam\'s origin $y$ position',
        ylabel='$y(a)$'
    )
    ax = fig.add_subplot(2, 3, 3)  # Initialize z(a) subplot
    do_position_subplot(
        fig, ax, arec['beam_origin_z'],
        label='$z(a)$',
        title='A-Beam\'s origin $z$ position',
        ylabel='$z(a)$'
    )
    ax = fig.add_subplot(2, 3, 4)  # Initialize x(b) subplot
    do_position_subplot(
        fig, ax, brec['beam_origin_x'],
        label='$x(b)$',
        title='B-Beam\'s origin $x$ position',
        ylabel='$x(b)$'
    )
    ax = fig.add_subplot(2, 3, 5)  # Initialize y(b) subplot
    do_position_subplot(
        fig, ax, brec['beam_origin_y'],
        label='$y(b)$',
        title='B-Beam\'s origin $y$ position',
        ylabel='$y(b)$'
    )
    ax = fig.add_subplot(2, 3, 6)  # Initialize z(b) subplot
    do_position_subplot(
        fig, ax, brec['beam_origin_z'],
        label='$z(b)$',
        title='B-Beam\'s origin $z$ position',
        ylabel='$z(b)$'
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(
        os.path.join(outdir, 'beam_origin.png')
    )
    fig.clear()
    plt.close(fig)


def do_attitude_subplot(
    fig, ax, x, label=None, title=None, ylabel=None, maxpoints=65536
):
    nsteps = len(x)
    steps = np.arange(1, nsteps+1)
    h = max(1, nsteps//maxpoints)  # Step size
    x, steps = x[::h], steps[::h]
    if title is not None:
        ax.set_title(title, fontsize=20)
    ax.plot(steps, x, label=label, lw=2, color='black')
    ax.set_xlabel('Simulation steps', fontsize=16)
    if ylabel is not None:
        ax.set_ylabel(ylabel, fontsize=16)
    ax.tick_params(axis='both', which='major', labelsize=14)
    ax.tick_params(axis='both', which='minor', labelsize=14)
    ax.legend(loc='upper right', fontsize=14)
    ax.grid('both')
    ax.set_axisbelow(True)


def do_platform_attitude_plots(arec, brec, outdir):
    # Validate platform attitude data
    if not validate_record('platform_roll', arec, 'a') or \
            not validate_record('platform_pitch', arec, 'a') or \
            not validate_record('platform_yaw', arec, 'a') or \
            not validate_record('platform_roll', brec, 'b') or \
            not validate_record('platform_pitch', brec, 'b') or \
            not validate_record('platform_yaw', brec, 'b'):
        print('Cannot do platform attitude plots')
        return

    # Do the platform attitude plots
    fig = init_figure()  # Initialize figure
    ax = fig.add_subplot(2, 3, 1)  # Initialize roll(a) subplot
    do_attitude_subplot(
        fig, ax, arec['platform_roll'],
        label='roll$(a)$',
        title='A-Platform\'s roll angle',
        ylabel='roll$(a)$'
    )
    ax = fig.add_subplot(2, 3, 2)  # Initialize pitch(a) subplot
    do_attitude_subplot(
        fig, ax, arec['platform_pitch'],
        label='pitch$(a)$',
        title='A-Platform\'s pitch angle',
        ylabel='pitch$(a)$'
    )
    ax = fig.add_subplot(2, 3, 3)  # Initialize yaw(a) subplot
    do_attitude_subplot(
        fig, ax, arec['platform_yaw'],
        label='yaw$(a)$',
        title='A-Platform\'s yaw angle',
        ylabel='yaw$(a)$'
    )
    ax = fig.add_subplot(2, 3, 4)  # Initialize roll(b) subplot
    do_attitude_subplot(
        fig, ax, brec['platform_roll'],
        label='roll$(b)$',
        title='B-Platform\'s roll angle',
        ylabel='roll$(b)$'
    )
    ax = fig.add_subplot(2, 3, 5)  # Initialize pitch(b) subplot
    do_attitude_subplot(
        fig, ax, brec['platform_pitch'],
        label='pitch$(b)$',
        title='B-Platform\'s pitch angle',
        ylabel='pitch$(b)$'
    )
    ax = fig.add_subplot(2, 3, 6)  # Initialize yaw(b) subplot
    do_attitude_subplot(
        fig, ax, brec['platform_yaw'],
        label='yaw$(b)$',
        title='B-Platform\'s yaw angle',
        ylabel='yaw$(b)$'
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(
        os.path.join(outdir, 'platform_attitude.png')
    )
    fig.clear()
    plt.close(fig)


def do_scanner_attitude_plots(arec, brec, outdir):
    # Validate scanner attitude data
    if not validate_record('scanner_roll', arec, 'a') or \
            not validate_record('scanner_pitch', arec, 'a') or \
            not validate_record('scanner_yaw', arec, 'a') or \
            not validate_record('scanner_roll', brec, 'b') or \
            not validate_record('scanner_pitch', brec, 'b') or \
            not validate_record('scanner_yaw', brec, 'b'):
        print('Cannot do scanner attitude plots')
        return

    # Do the scanner attitude plots
    fig = init_figure()  # Initialize figure
    ax = fig.add_subplot(2, 3, 1)  # Initialize roll(a) subplot
    do_attitude_subplot(
        fig, ax, arec['scanner_roll'],
        label='roll$(a)$',
        title='A-Scanner\'s roll angle',
        ylabel='roll$(a)$'
    )
    ax = fig.add_subplot(2, 3, 2)  # Initialize pitch(a) subplot
    do_attitude_subplot(
        fig, ax, arec['scanner_pitch'],
        label='pitch$(a)$',
        title='A-Scanner\'s pitch angle',
        ylabel='pitch$(a)$'
    )
    ax = fig.add_subplot(2, 3, 3)  # Initialize yaw(a) subplot
    do_attitude_subplot(
        fig, ax, arec['scanner_yaw'],
        label='yaw$(a)$',
        title='A-Scanner\'s yaw angle',
        ylabel='yaw$(a)$'
    )
    ax = fig.add_subplot(2, 3, 4)  # Initialize roll(b) subplot
    do_attitude_subplot(
        fig, ax, brec['scanner_roll'],
        label='roll$(b)$',
        title='B-Scanner\'s roll angle',
        ylabel='roll$(b)$'
    )
    ax = fig.add_subplot(2, 3, 5)  # Initialize pitch(b) subplot
    do_attitude_subplot(
        fig, ax, brec['scanner_pitch'],
        label='pitch$(b)$',
        title='B-Scanner\'s pitch angle',
        ylabel='pitch$(b)$'
    )
    ax = fig.add_subplot(2, 3, 6)  # Initialize yaw(b) subplot
    do_attitude_subplot(
        fig, ax, brec['scanner_yaw'],
        label='yaw$(b)$',
        title='B-Scanner\'s yaw angle',
        ylabel='yaw$(b)$'
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(
        os.path.join(outdir, 'scanner_attitude.png')
    )
    fig.clear()
    plt.close(fig)


def do_scanner_head_attitude_plots(arec, brec, outdir):
    # Validate scanner head attitude data
    if not validate_record('scanner_head_roll', arec, 'a') or \
            not validate_record('scanner_head_pitch', arec, 'a') or \
            not validate_record('scanner_head_yaw', arec, 'a') or \
            not validate_record('scanner_head_roll', brec, 'b') or \
            not validate_record('scanner_head_pitch', brec, 'b') or \
            not validate_record('scanner_head_yaw', brec, 'b'):
        print('Cannot do scanner head attitude plots')
        return

    # Do the scanner head attitude plots
    fig = init_figure()  # Initialize figure
    ax = fig.add_subplot(2, 3, 1)  # Initialize roll(a) subplot
    do_attitude_subplot(
        fig, ax, arec['scanner_head_roll'],
        label='roll$(a)$',
        title='A-Scanner\'s head roll angle',
        ylabel='roll$(a)$'
    )
    ax = fig.add_subplot(2, 3, 2)  # Initialize pitch(a) subplot
    do_attitude_subplot(
        fig, ax, arec['scanner_head_pitch'],
        label='pitch$(a)$',
        title='A-Scanner\'s head pitch angle',
        ylabel='pitch$(a)$'
    )
    ax = fig.add_subplot(2, 3, 3)  # Initialize yaw(a) subplot
    do_attitude_subplot(
        fig, ax, arec['scanner_head_yaw'],
        label='yaw$(a)$',
        title='A-Scanner\'s head yaw angle',
        ylabel='yaw$(a)$'
    )
    ax = fig.add_subplot(2, 3, 4)  # Initialize roll(b) subplot
    do_attitude_subplot(
        fig, ax, brec['scanner_head_roll'],
        label='roll$(b)$',
        title='B-Scanner\'s head roll angle',
        ylabel='roll$(b)$'
    )
    ax = fig.add_subplot(2, 3, 5)  # Initialize pitch(b) subplot
    do_attitude_subplot(
        fig, ax, brec['scanner_head_pitch'],
        label='pitch$(b)$',
        title='B-Scanner\'s head pitch angle',
        ylabel='pitch$(b)$'
    )
    ax = fig.add_subplot(2, 3, 6)  # Initialize yaw(b) subplot
    do_attitude_subplot(
        fig, ax, brec['scanner_head_yaw'],
        label='yaw$(b)$',
        title='B-Scanner\'s head yaw angle',
        ylabel='yaw$(b)$'
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(
        os.path.join(outdir, 'scanner_head_attitude.png')
    )
    fig.clear()
    plt.close(fig)


def do_beam_attitude_plots(arec, brec, outdir):
    # Validate beam attitude data
    if not validate_record('beam_roll', arec, 'a') or \
            not validate_record('beam_pitch', arec, 'a') or \
            not validate_record('beam_yaw', arec, 'a') or \
            not validate_record('beam_roll', brec, 'b') or \
            not validate_record('beam_pitch', brec, 'b') or \
            not validate_record('beam_yaw', brec, 'b'):
        print('Cannot do beam attitude plots')
        return

    # Do the beam attitude plots
    fig = init_figure()  # Initialize figure
    ax = fig.add_subplot(2, 3, 1)  # Initialize roll(a) subplot
    do_attitude_subplot(
        fig, ax, arec['beam_roll'],
        label='roll$(a)$',
        title='A-Beam\'s roll angle',
        ylabel='roll$(a)$'
    )
    ax = fig.add_subplot(2, 3, 2)  # Initialize pitch(a) subplot
    do_attitude_subplot(
        fig, ax, arec['beam_pitch'],
        label='pitch$(a)$',
        title='A-Beam\'s pitch angle',
        ylabel='pitch$(a)$'
    )
    ax = fig.add_subplot(2, 3, 3)  # Initialize yaw(a) subplot
    do_attitude_subplot(
        fig, ax, arec['beam_yaw'],
        label='yaw$(a)$',
        title='A-Beam\'s yaw angle',
        ylabel='yaw$(a)$'
    )
    ax = fig.add_subplot(2, 3, 4)  # Initialize roll(b) subplot
    do_attitude_subplot(
        fig, ax, brec['beam_roll'],
        label='roll$(b)$',
        title='B-Beam\'s roll angle',
        ylabel='roll$(b)$'
    )
    ax = fig.add_subplot(2, 3, 5)  # Initialize pitch(b) subplot
    do_attitude_subplot(
        fig, ax, brec['beam_pitch'],
        label='pitch$(b)$',
        title='B-Beam\'s pitch angle',
        ylabel='pitch$(b)$'
    )
    ax = fig.add_subplot(2, 3, 6)  # Initialize yaw(b) subplot
    do_attitude_subplot(
        fig, ax, brec['beam_yaw'],
        label='yaw$(b)$',
        title='B-Beam\'s yaw angle',
        ylabel='yaw$(b)$'
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(
        os.path.join(outdir, 'beam_attitude.png')
    )
    fig.clear()
    plt.close(fig)


def do_deflector_emitting_attitude_plots(arec, brec, outdir):
    # Validate deflector emitting attitude data
    if not validate_record('deflector_emitting_roll', arec, 'a') or \
            not validate_record('deflector_emitting_pitch', arec, 'a') or \
            not validate_record('deflector_emitting_yaw', arec, 'a') or \
            not validate_record('deflector_emitting_roll', brec, 'b') or \
            not validate_record('deflector_emitting_pitch', brec, 'b') or \
            not validate_record('deflector_emitting_yaw', brec, 'b'):
        print('Cannot do deflector emitting attitude plots')
        return

    # Do the deflector emitting attitude plots
    fig = init_figure()  # Initialize figure
    ax = fig.add_subplot(2, 3, 1)  # Initialize roll(a) subplot
    do_attitude_subplot(
        fig, ax, arec['deflector_emitting_roll'],
        label='roll$(a)$',
        title='A-Deflector\'s emitting roll angle',
        ylabel='roll$(a)$'
    )
    ax = fig.add_subplot(2, 3, 2)  # Initialize pitch(a) subplot
    do_attitude_subplot(
        fig, ax, arec['deflector_emitting_pitch'],
        label='pitch$(a)$',
        title='A-Deflector\'s emitting pitch angle',
        ylabel='pitch$(a)$'
    )
    ax = fig.add_subplot(2, 3, 3)  # Initialize yaw(a) subplot
    do_attitude_subplot(
        fig, ax, arec['deflector_emitting_yaw'],
        label='yaw$(a)$',
        title='A-Deflector\'s emitting yaw angle',
        ylabel='yaw$(a)$'
    )
    ax = fig.add_subplot(2, 3, 4)  # Initialize roll(b) subplot
    do_attitude_subplot(
        fig, ax, brec['deflector_emitting_roll'],
        label='roll$(b)$',
        title='B-Deflector\'s emitting roll angle',
        ylabel='roll$(b)$'
    )
    ax = fig.add_subplot(2, 3, 5)  # Initialize pitch(b) subplot
    do_attitude_subplot(
        fig, ax, brec['deflector_emitting_pitch'],
        label='pitch$(b)$',
        title='B-Deflector\'s emitting pitch angle',
        ylabel='pitch$(b)$'
    )
    ax = fig.add_subplot(2, 3, 6)  # Initialize yaw(b) subplot
    do_attitude_subplot(
        fig, ax, brec['deflector_emitting_yaw'],
        label='yaw$(b)$',
        title='B-Deflector\'s emitting yaw angle',
        ylabel='yaw$(b)$'
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(
        os.path.join(outdir, 'deflector_emitting_attitude.png')
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
