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
    subray_sim = read_record(os.path.join(
        path, 'subray_sim.csv'
    ), sep)
    # Return key-word records
    return {
        # Intensity calculation records
        'incidence_angle_rad': intensity_calc[:, 3],
        'target_range_m': intensity_calc[:, 4],
        'target_area_m2': intensity_calc[:, 5],
        'radius_m': intensity_calc[:, 6],
        'bdrf': intensity_calc[:, 7],
        'cross_section': intensity_calc[:, 8],
        'received_power': intensity_calc[:, 9],
        # Subray simulation records
        'subray_hit': subray_sim[:, 0].astype(bool),
        'radius_step': subray_sim[:, 1],
        'circle_steps': subray_sim[:, 2],
        'circle_step': subray_sim[:, 3],
        'divergence_angle_rad': subray_sim[:, 4],
        'ray_dir_norm': subray_sim[:, 5],
        'subray_dir_norm': subray_sim[:, 6],
        'ray_subray_angle_rad': subray_sim[:, 7],
        'ray_subray_sign_check': subray_sim[:, 8],
        'subray_tmin': subray_sim[:, 9],
        'subray_tmax': subray_sim[:, 10],
        'subray_rt_pos_dir': subray_sim[:, 11],  # Ray-tracing positive dir.
        'subray_rt_neg_dir': subray_sim[:, 12],  # Ray-tracing negative dir.
        'subray_rt_par_dir': subray_sim[:, 13],  # Ray-tracing parallel dir.
        'subray_rt_no_second': subray_sim[:, 14],  # Ray-tracing no second half
        'subray_rt_no_first': subray_sim[:, 15],  # Ray-tracing no first half
        'subray_rt_both': subray_sim[:, 16],  # Ray-tracing both sides
        'subray_rt_both2': subray_sim[:, 17]  # Ray-tracing both, second try
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
    do_subray_hit_plots(arec, brec, outdir)
    do_ray_subray_plots(arec, brec, outdir)
    do_raytracing_plots(arec, brec, outdir)


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

    # Do the incidence angle plots (rads)
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
        os.path.join(outdir, 'incidence_angle_distribution_rad.png')
    )
    fig.clear()
    plt.close(fig)
    # Do the incidence angle plots (degrees)
    fig = init_figure()  # Initialize figure
    ax = fig.add_subplot(2, 2, 1)  # Initialize phi(a) subplot
    do_incidence_angle_subplot(
        fig, ax, arec['incidence_angle_rad']*180/np.pi,
        label='$\\varphi(a)$',
        title='A-Incidence angle ($\\varphi$) in rad',
        xlabel='$\\varphi(a)$',
        ylabel='cases'
    )
    ax = fig.add_subplot(2, 2, 3)  # Initialize phi(a) log subplot
    do_incidence_angle_subplot(
        fig, ax, arec['incidence_angle_rad']*180/np.pi,
        label='$\\varphi(a)$',
        title='A-Incidence angle ($\\varphi$) in rad (logarithmic)',
        xlabel='$\\varphi(a)$',
        ylabel='cases',
        log=True
    )
    ax = fig.add_subplot(2, 2, 2)  # Initialize phi(b) subplot
    do_incidence_angle_subplot(
        fig, ax, brec['incidence_angle_rad']*180/np.pi,
        label='$\\varphi(b)$',
        title='B-Incidence angle ($\\varphi$) in rad',
        xlabel='$\\varphi(b)$',
        ylabel='cases'
    )
    ax = fig.add_subplot(2, 2, 4)  # Initialize phi(a) log subplot
    do_incidence_angle_subplot(
        fig, ax, brec['incidence_angle_rad']*180/np.pi,
        label='$\\varphi(b)$',
        title='B-Incidence angle ($\\varphi$) in rad (logarithmic)',
        xlabel='$\\varphi(b)$',
        ylabel='cases',
        log=True
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(
        os.path.join(outdir, 'incidence_angle_distribution_degrees.png')
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
    # Plot by radians
    _do_by_incidence_angle_plots(
        arec['incidence_angle_rad'],
        brec['incidence_angle_rad'],
        arec,
        brec,
        outdir,
        fname='plot_by_incidence_angle_rad.png'
    )
    # Plot by degrees
    _do_by_incidence_angle_plots(
        arec['incidence_angle_rad']*180/np.pi,
        brec['incidence_angle_rad']*180/np.pi,
        arec,
        brec,
        outdir,
        fname='plot_by_incidence_angle_degrees.png'
    )


def _do_by_incidence_angle_plots(
    incidence_angle_a, incidence_angle_b, arec, brec, outdir, fname
):
    # Do the "by incidence angle" plots
    fig = init_figure()  # Initialize figure
    ax = fig.add_subplot(3, 4, 1)  # Initialize target range A subplot
    do_y_by_x_subplot(
        fig, ax, incidence_angle_a, arec['target_range_m'],
        title='A-Target range (m)',
        xlabel='Incidence angle (rad)',
        ylabel='Target range (m)',
        color='black'
    )
    ax = fig.add_subplot(3, 4, 2)  # Initialize target area A subplot
    do_y_by_x_subplot(
        fig, ax, incidence_angle_a, arec['target_area_m2'],
        title='A-Target area ($m^2$)',
        xlabel='Incidence angle (rad)',
        ylabel='Target area ($m^2$)',
        color='tab:blue'
    )
    ax = fig.add_subplot(3, 4, 5)  # Initialize radius A subplot
    do_y_by_x_subplot(
        fig, ax, incidence_angle_a, arec['radius_m'],
        title='A-Radius (m)',
        xlabel='Incidence angle (rad)',
        ylabel='Radius (m)',
        color='tab:red'
    )
    ax = fig.add_subplot(3, 4, 6)  # Initialize BDRF A subplot
    do_y_by_x_subplot(
        fig, ax, incidence_angle_a, arec['bdrf'],
        title='A-BDRF',
        xlabel='Incidence angle (rad)',
        ylabel='BDRF',
        color='tab:green'
    )
    ax = fig.add_subplot(3, 4, 9)  # Initialize Cross-section A subplot
    do_y_by_x_subplot(
        fig, ax, incidence_angle_a, arec['cross_section'],
        title='A-Cross-section ($m^2$)',
        xlabel='Incidence angle (rad)',
        ylabel='Cross-section ($m^2$)',
        color='tab:orange'
    )
    ax = fig.add_subplot(3, 4, 10)  # Initialize received power A subplot
    do_y_by_x_subplot(
        fig, ax, incidence_angle_a, arec['received_power'],
        title='A-Received power',
        xlabel='Incidence angle (rad)',
        ylabel='Received power',
        color='tab:purple'
    )
    ax = fig.add_subplot(3, 4, 3)  # Initialize target range B subplot
    do_y_by_x_subplot(
        fig, ax, incidence_angle_b, brec['target_range_m'],
        title='B-Target range (m)',
        xlabel='Incidence angle (rad)',
        ylabel='Target range (m)',
        color='black'
    )
    ax = fig.add_subplot(3, 4, 4)  # Initialize target area B subplot
    do_y_by_x_subplot(
        fig, ax, incidence_angle_b, brec['target_area_m2'],
        title='B-Target area ($m^2$)',
        xlabel='Incidence angle (rad)',
        ylabel='Target area ($m^2$)',
        color='tab:blue'
    )
    ax = fig.add_subplot(3, 4, 7)  # Initialize radius B subplot
    do_y_by_x_subplot(
        fig, ax, incidence_angle_b, brec['radius_m'],
        title='B-Radius (m)',
        xlabel='Incidence angle (rad)',
        ylabel='Radius (m)',
        color='tab:red'
    )
    ax = fig.add_subplot(3, 4, 8)  # Initialize BDRF B subplot
    do_y_by_x_subplot(
        fig, ax, incidence_angle_b, brec['bdrf'],
        title='B-BDRF',
        xlabel='Incidence angle (rad)',
        ylabel='BDRF',
        color='tab:green'
    )
    ax = fig.add_subplot(3, 4, 11)  # Initialize Cross-section B subplot
    do_y_by_x_subplot(
        fig, ax, incidence_angle_b, brec['cross_section'],
        title='B-Cross-section ($m^2$)',
        xlabel='Incidence angle (rad)',
        ylabel='Cross-section ($m^2$)',
        color='tab:orange'
    )
    ax = fig.add_subplot(3, 4, 12)  # Initialize received power B subplot
    do_y_by_x_subplot(
        fig, ax, incidence_angle_b, brec['received_power'],
        title='B-Received power',
        xlabel='Incidence angle (rad)',
        ylabel='Received power',
        color='tab:purple'
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(
        os.path.join(outdir, fname)
    )
    fig.clear()
    plt.close(fig)


def do_subray_hit_subplot_hist2d(
    fig, ax, x, y, title=None, xlabel=None, ylabel=None, bins="auto"
):
    if title is not None:
        ax.set_title(title, fontsize=15)
    if bins == "auto":
        bins = [len(np.unique(x)), len(np.unique(y))]
    hist2d = ax.hist2d(
        x, y, bins=bins, cmap='viridis',
        weights=100*np.ones_like(x)/len(x),
        edgecolors='black'
    )
    fig.colorbar(hist2d[3])
    if xlabel is not None:
        ax.set_xlabel(xlabel, fontsize=14)
    if ylabel is not None:
        ax.set_ylabel(ylabel, fontsize=14)
    ax.tick_params(axis='both', which='both', labelsize=12)
    ax.grid('both')
    ax.set_axisbelow(True)


def do_subray_hit_subplot_hist(
    fig, ax, hit, x, title=None, xlabel=None, ylabel=None, bins=7,
    relative=False
):
    # TODO Rethink : Implement
    if title is not None:
        ax.set_title(title, fontsize=15)
    x_hit = x[hit]
    x_nohit = x[~hit]
    weights = [
        100*np.ones_like(x_hit)/len(x_hit),
        100*np.ones_like(x_nohit)/len(x_nohit)
    ] if relative else None
    hist = ax.hist(
        [x_hit, x_nohit], bins=bins, label=['hit', 'miss'], weights=weights
    )
    if xlabel is not None:
        ax.set_xlabel(xlabel, fontsize=14)
    if ylabel is not None:
        ax.set_ylabel(ylabel, fontsize=14)
    ax.tick_params(axis='both', which='both', labelsize=14)
    ax.legend(loc='upper right', fontsize=12)
    ax.grid('both')
    ax.set_axisbelow(True)


def do_subray_hit_plots(arec, brec, outdir):
    # Validate subray hit data
    if(not validate_record('subray_hit', arec, 'a') or
        not validate_record('radius_step', arec, 'a') or
        not validate_record('circle_steps', arec, 'a') or
        not validate_record('circle_step', arec, 'a') or
        not validate_record('divergence_angle_rad', arec, 'a') or
        not validate_record('subray_hit', brec, 'b') or
        not validate_record('radius_step', brec, 'b') or
        not validate_record('circle_steps', brec, 'b') or
        not validate_record('circle_step', brec, 'b') or
        not validate_record('divergence_angle_rad', brec, 'b')
       ):
        print('Cannot do subray hit plots')
        return

    # Do the subray hit plots
    fig = init_figure()  # Initialize figure
    # CASE A
    ax = fig.add_subplot(4, 5, 1)  # Initialize hit2Dhist on (radstep,circstep)
    do_subray_hit_subplot_hist2d(
        fig, ax,
        arec['circle_step'][arec['subray_hit']],
        arec['radius_step'][arec['subray_hit']],
        title='Hit distribution (100%) (A)',
    )
    ax = fig.add_subplot(4, 5, 2)  # Initialize a hist on radius step by hit
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['radius_step'],
        ylabel='Absolute'
    )
    ax = fig.add_subplot(4, 5, 3)  # Initialize a hist on circle steps by hit
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['circle_steps'],
    )
    ax = fig.add_subplot(4, 5, 4)  # Initialize a hist on circle step by hit
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['circle_step'],
    )
    ax = fig.add_subplot(4, 5, 5)  # Initialize a hist on div. angle by hit
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'],
        1e03*arec['divergence_angle_rad']*180/np.pi,
    )
    ax = fig.add_subplot(4, 5, 6)  # Initialize non-hit 2D hist on (rs, cs)
    do_subray_hit_subplot_hist2d(
        fig, ax,
        arec['circle_step'][~arec['subray_hit']],
        arec['radius_step'][~arec['subray_hit']],
        title='No-hit distribution (100%)',
        xlabel='Circle step',
        ylabel='Radius step'
    )
    ax = fig.add_subplot(4, 5, 7)  # Initialize a hist on radius step by hit
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['radius_step'],
        ylabel='Relative ($100\\%$)',
        relative=True,
        xlabel='Radius step'
    )
    ax = fig.add_subplot(4, 5, 8)  # Initialize a hist on circle steps by hit
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['circle_steps'],
        relative=True,
        xlabel='Circle steps'
    )
    ax = fig.add_subplot(4, 5, 9)  # Initialize a hist on circle step by hit
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['circle_step'],
        relative=True,
        xlabel='Circle step'
    )
    ax = fig.add_subplot(4, 5, 10)  # Initialize a hist on div. angle by hit
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'],
        1e03*arec['divergence_angle_rad']*180/np.pi,
        relative=True,
        xlabel='Divergence angle (deg $\\times 10^{-3}$)'
    )
    # CASE B
    ax = fig.add_subplot(4, 5, 11)  # Initialize hit2Dhist on (radstep,circstep)
    do_subray_hit_subplot_hist2d(
        fig, ax,
        brec['circle_step'][brec['subray_hit']],
        brec['radius_step'][brec['subray_hit']],
        title='Hit distribution (100%) (B)',
    )
    ax = fig.add_subplot(4, 5, 12)  # Initialize a hist on radius step by hit
    do_subray_hit_subplot_hist(
        fig, ax, brec['subray_hit'], brec['radius_step'],
        ylabel='Absolute'
    )
    ax = fig.add_subplot(4, 5, 13)  # Initialize a hist on circle steps by hit
    do_subray_hit_subplot_hist(
        fig, ax, brec['subray_hit'], brec['circle_steps'],
    )
    ax = fig.add_subplot(4, 5, 14)  # Initialize a hist on circle step by hit
    do_subray_hit_subplot_hist(
        fig, ax, brec['subray_hit'], brec['circle_step'],
    )
    ax = fig.add_subplot(4, 5, 15)  # Initialize a hist on div. angle by hit
    do_subray_hit_subplot_hist(
        fig, ax, brec['subray_hit'],
        1e03*brec['divergence_angle_rad']*180/np.pi,
    )
    ax = fig.add_subplot(4, 5, 16)  # Initialize non-hit 2D hist on (rs, cs)
    do_subray_hit_subplot_hist2d(
        fig, ax,
        brec['circle_step'][~brec['subray_hit']],
        brec['radius_step'][~brec['subray_hit']],
        title='No-hit distribution (100%)',
        xlabel='Circle step',
        ylabel='Radius step'
    )
    ax = fig.add_subplot(4, 5, 17)  # Initialize a hist on radius step by hit
    do_subray_hit_subplot_hist(
        fig, ax, brec['subray_hit'], brec['radius_step'],
        ylabel='Relative ($100\\%$)',
        relative=True,
        xlabel='Radius step'
    )
    ax = fig.add_subplot(4, 5, 18)  # Initialize a hist on circle steps by hit
    do_subray_hit_subplot_hist(
        fig, ax, brec['subray_hit'], brec['circle_steps'],
        relative=True,
        xlabel='Circle steps'
    )
    ax = fig.add_subplot(4, 5, 19)  # Initialize a hist on circle step by hit
    do_subray_hit_subplot_hist(
        fig, ax, brec['subray_hit'], brec['circle_step'],
        relative=True,
        xlabel='Circle step'
    )
    ax = fig.add_subplot(4, 5, 20)  # Initialize a hist on div. angle by hit
    do_subray_hit_subplot_hist(
        fig, ax, brec['subray_hit'],
        1e03*brec['divergence_angle_rad']*180/np.pi,
        relative=True,
        xlabel='Divergence angle (deg $\\times 10^{-3}$)'
    )
    # TODO Rethink : Implement
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(os.path.join(outdir, 'subray_hit.png'))
    fig.clear()
    plt.close(fig)


def do_ray_subray_plots(arec, brec, outdir):
    # Validate ray subray data
    if(
        not validate_record('ray_dir_norm', arec, 'a') or
        not validate_record('subray_dir_norm', arec, 'a') or
        not validate_record('ray_subray_angle_rad', arec, 'a') or
        not validate_record('ray_subray_sign_check', arec, 'a') or
        not validate_record('subray_tmin', arec, 'a') or
        not validate_record('subray_tmax', arec, 'a') or
        not validate_record('ray_dir_norm', brec, 'b') or
        not validate_record('subray_dir_norm', brec, 'b') or
        not validate_record('ray_subray_angle_rad', brec, 'b') or
        not validate_record('ray_subray_sign_check', brec, 'b') or
        not validate_record('subray_tmin', brec, 'b') or
        not validate_record('subray_tmax', brec, 'b')
      ):
        print('Cannot do ray-subray plots')
        return
    # Do the ray subray plots
    fig = init_figure()  # Initialize figure
    # CASE A
    ax = fig.add_subplot(3, 4, 1)  # Initialize ray norm subplot
    do_incidence_angle_subplot(
        fig, ax, arec['ray_dir_norm'],
        xlabel='Ray direction norm'
    )
    ax = fig.add_subplot(3, 4, 2)  # Initialize subray norm subplot
    do_incidence_angle_subplot(
        fig, ax, arec['subray_dir_norm'],
        xlabel='Subay direction norm'
    )
    ax = fig.add_subplot(3, 4, 5)  # Initialize ray-subray angle subplot
    do_incidence_angle_subplot(
        fig, ax, arec['ray_subray_angle_rad']*180/np.pi,
        xlabel='Ray-subray angle (deg)'
    )
    ax = fig.add_subplot(3, 4, 6)  # Initialize ray-subray sign check subplot
    do_incidence_angle_subplot(
        fig, ax, arec['ray_subray_sign_check'],
        xlabel='Sign equality check'
    )
    ax = fig.add_subplot(3, 4, 9)  # Initialize subray tmin subplot
    do_incidence_angle_subplot(
        fig, ax, arec['subray_tmin'],
        xlabel='Subray $t_{\\mathrm{min}}$'
    )
    ax = fig.add_subplot(3, 4, 10)  # Initialize subray tmin subplot
    do_incidence_angle_subplot(
        fig, ax, arec['subray_tmax'],
        xlabel='Subray $t_{\\mathrm{max}}$'
    )
    # CASE B
    ax = fig.add_subplot(3, 4, 3)  # Initialize ray norm subplot
    do_incidence_angle_subplot(
        fig, ax, brec['ray_dir_norm'],
        xlabel='Ray direction norm'
    )
    ax = fig.add_subplot(3, 4, 4)  # Initialize subray norm subplot
    do_incidence_angle_subplot(
        fig, ax, brec['subray_dir_norm'],
        xlabel='Subay direction norm'
    )
    ax = fig.add_subplot(3, 4, 7)  # Initialize ray-subray angle subplot
    do_incidence_angle_subplot(
        fig, ax, brec['ray_subray_angle_rad']*180/np.pi,
        xlabel='Ray-subray angle (deg)'
    )
    ax = fig.add_subplot(3, 4, 8)  # Initialize ray-subray sign check subplot
    do_incidence_angle_subplot(
        fig, ax, brec['ray_subray_sign_check'],
        xlabel='Sign equality check'
    )
    ax = fig.add_subplot(3, 4, 11)  # Initialize subray tmin subplot
    do_incidence_angle_subplot(
        fig, ax, brec['subray_tmin'],
        xlabel='Subray $t_{\\mathrm{min}}$'
    )
    ax = fig.add_subplot(3, 4, 12)  # Initialize subray tmin subplot
    do_incidence_angle_subplot(
        fig, ax, brec['subray_tmax'],
        xlabel='Subray $t_{\\mathrm{max}}$'
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(
        os.path.join(outdir, 'ray_subray.png')
    )
    fig.clear()
    plt.close(fig)


def do_raytracing_plots(arec, brec, outdir):
    # TODO Rethink : Implement
    if(
        not validate_record('subray_hit', arec, 'a') or
        not validate_record('subray_rt_pos_dir', arec, 'a') or
        not validate_record('subray_rt_neg_dir', arec, 'a') or
        not validate_record('subray_rt_par_dir', arec, 'a') or
        not validate_record('subray_rt_no_second', arec, 'a') or
        not validate_record('subray_rt_no_first', arec, 'a') or
        not validate_record('subray_rt_both', arec, 'a') or
        not validate_record('subray_rt_both2', arec, 'a') or
        not validate_record('subray_hit', brec, 'b') or
        not validate_record('subray_rt_pos_dir', brec, 'b') or
        not validate_record('subray_rt_neg_dir', brec, 'b') or
        not validate_record('subray_rt_par_dir', brec, 'b') or
        not validate_record('subray_rt_no_second', brec, 'b') or
        not validate_record('subray_rt_no_first', brec, 'b') or
        not validate_record('subray_rt_both', brec, 'b') or
        not validate_record('subray_rt_both2', brec, 'b')
      ):
        print('Cannot do ray-tracing plots')
        return
    # Do the ray-tracing plots
    fig = init_figure()  # Initialize figure
    # CASE A
    ax = fig.add_subplot(4, 7, 1)  # Initialize positive direction hist
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['subray_rt_pos_dir'],
        ylabel='Absolute'
    )
    ax = fig.add_subplot(4, 7, 2)  # Initialize negative direction hist
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['subray_rt_neg_dir'],
    )
    ax = fig.add_subplot(4, 7, 3)  # Initialize parallel direction hist
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['subray_rt_par_dir'],
    )
    ax = fig.add_subplot(4, 7, 4)  # Initialize no second half hist
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['subray_rt_no_second'],
    )
    ax = fig.add_subplot(4, 7, 5)  # Initialize no second half hist
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['subray_rt_no_first'],
    )
    ax = fig.add_subplot(4, 7, 6)  # Initialize both sides hist
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['subray_rt_both'],
    )
    ax = fig.add_subplot(4, 7, 7)  # Initialize both sides hist
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['subray_rt_both2'],
    )
    ax = fig.add_subplot(4, 7, 8)  # Initialize positive direction hist
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['subray_rt_pos_dir'],
        relative=True,
        xlabel='Positive direction',
        ylabel='Relative (100%)'
    )
    ax = fig.add_subplot(4, 7, 9)  # Initialize negative direction hist
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['subray_rt_neg_dir'],
        relative=True,
        xlabel='Negative direction',
    )
    ax = fig.add_subplot(4, 7, 10)  # Initialize parallel direction hist
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['subray_rt_par_dir'],
        relative=True,
        xlabel='Parallel direction',
    )
    ax = fig.add_subplot(4, 7, 11)  # Initialize no second half hist
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['subray_rt_no_second'],
        relative=True,
        xlabel='No second half',
    )
    ax = fig.add_subplot(4, 7, 12)  # Initialize no second half hist
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['subray_rt_no_first'],
        relative=True,
        xlabel='No first half',
    )
    ax = fig.add_subplot(4, 7, 13)  # Initialize both sides hist
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['subray_rt_both'],
        relative=True,
        xlabel='Both sides',
    )
    ax = fig.add_subplot(4, 7, 14)  # Initialize both sides hist
    do_subray_hit_subplot_hist(
        fig, ax, arec['subray_hit'], arec['subray_rt_both2'],
        relative=True,
        xlabel='Both sides ($2^{\mathrm{nd}}$ try)',
    )
    # CASE B
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(
        os.path.join(outdir, 'subray_ray_tracing.png')
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
