import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl


# ---  FUNCTIONS  --- #
# ------------------- #
def print_help():
    print(
        """
Input arguments:
    1 -> Path to the first directory containing simulation records
    2 -> Path to the second directory containing simulation records
    3 -> Path to the directory where plots will be stored
"""
    )


def parse_args(helpf=print_help):
    """Parse input arguments. Raise an exception if not correct arguments were
    given"""
    if len(sys.argv) == 1:
        helpf()
        exit(0)
    elif len(sys.argv) < 4:
        raise Exception(
            "{m} arguments were given but 3 are required".format(m=len(sys.argv) - 1)
        )
    dira_path = sys.argv[1]
    if not validate_directory(dira_path):
        raise Exception(
            'The directory "{d}"\n'
            "was given as the first directory of records, but it is not valid"
        )
    dirb_path = sys.argv[2]
    if not validate_directory(dirb_path):
        raise Exception(
            'The directory "{d}"\n'
            "was given as the second directory of records, but it is not valid"
        )
    dirout_path = sys.argv[3]
    if not validate_directory(dirout_path):
        raise Exception(
            'The directory "{d}"\n'
            "was given as the third directory for plots, but it is not valid"
        )
    return {"dira_path": dira_path, "dirb_path": dirb_path, "dirout_path": dirout_path}


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


def read_records(path, sep=","):
    """Read all record files contained in the directory pointed by given
    path"""
    # Read vectorial records
    intensity_calc = read_record(os.path.join(path, "intensity_calc.csv"), sep)
    intensity_indices = read_record(
        os.path.join(path, "intensity_calc_indices.csv"), sep, dtype=int
    )
    subray_sim = read_record(os.path.join(path, "subray_sim.csv"), sep)
    # Return key-word records
    return {
        # Intensity calculation records
        "incidence_angle_rad": intensity_calc[:, 3],
        "target_range_m": intensity_calc[:, 4],
        "target_area_m2": intensity_calc[:, 5],
        "radius_m": intensity_calc[:, 6],
        "bdrf": intensity_calc[:, 7],
        "cross_section": intensity_calc[:, 8],
        "received_power": intensity_calc[:, 9],
        "emitted_power": intensity_calc[:, 11],
        "radius_step": intensity_calc[:, 12],
        # Intensity calculation indices
        "ray_idx": intensity_indices,
        # Subray simulation records
        "subray_hit": subray_sim[:, 0].astype(bool),
        "divergence_angle_rad": subray_sim[:, 1],
        "ray_dir_norm": subray_sim[:, 2],
        "subray_dir_norm": subray_sim[:, 3],
        "ray_subray_angle_rad": subray_sim[:, 4],
        "ray_subray_sign_check": subray_sim[:, 5],
        "subray_tmin": subray_sim[:, 6],
        "subray_tmax": subray_sim[:, 7],
        "subray_dir_x": subray_sim[:, 8],
        "subray_dir_y": subray_sim[:, 9],
        "subray_dir_z": subray_sim[:, 10],
        "ray_dir_x": subray_sim[:, 11],
        "ray_dir_y": subray_sim[:, 12],
        "ray_dir_z": subray_sim[:, 13],
    }


def read_record(path, sep, dtype=float):
    """Read given record file
    :param path: The path to the record file to be read
    :param sep: The separator used in the record file
    :return: None if the record file could not be read, the data as a numpy
        array otherwise
    """
    if os.path.exists(path) and os.path.isfile(path):
        return np.loadtxt(path, delimiter=sep, dtype=dtype)
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
    do_by_incidence_angle_plots(arec, brec, outdir, emitted_power=True)
    do_subray_hit_plots(arec, brec, outdir)
    do_ray_subray_plots(arec, brec, outdir)
    do_ray_subray_dir_plots(arec, brec, outdir)
    do_energy_plots(arec, brec, outdir)


def validate_record(key, rec, recid):
    """Check that the record with given key is available
    :return: True if the record is valid (available data), False otherwise
    """
    if key not in rec or rec.get(key, None) is None:
        print(
            'Record "{key}" is not available for {recid} records'.format(
                key=key, recid=recid
            )
        )
        return False
    return True


def init_figure(figsize=(20, 12), constrained_layout=False):
    """Initialize a matplotlib's figure context"""
    fig = plt.figure(figsize=figsize, constrained_layout=constrained_layout)
    return fig


def do_incidence_angle_subplot(
    fig,
    ax,
    phi,
    label=None,
    title=None,
    xlabel=None,
    ylabel=None,
    bins=32,
    log=False,
    relative=False,
    label_fontsize=16,
):
    if title is not None:
        ax.set_title(title, fontsize=20)
    weights = 100 * np.ones_like(phi) / len(phi) if relative else None
    hist = ax.hist(phi, bins=bins, label=label, log=log, weights=weights)
    ax.axvline(x=np.mean(phi), color="tab:orange", lw=3, label="$\\mu$")
    if xlabel is not None:
        ax.set_xlabel(xlabel, fontsize=label_fontsize)
    if ylabel is not None:
        ax.set_ylabel(ylabel, fontsize=label_fontsize)
    ax.tick_params(axis="both", which="both", labelsize=14)
    ax.legend(loc="upper right", fontsize=14)
    ax.grid("both")
    ax.set_axisbelow(True)


def do_incidence_angle_plots(arec, brec, outdir):
    # Validate incidence angle data
    if not validate_record("incidence_angle_rad", arec, "a") or not validate_record(
        "incidence_angle_rad", brec, "b"
    ):
        print("Cannot do incidence angle plots")
        return
    # Do the incidence angle plots (rads)
    fig = init_figure()  # Initialize figure
    ax = fig.add_subplot(2, 2, 1)  # Initialize phi(a) subplot
    do_incidence_angle_subplot(
        fig,
        ax,
        arec["incidence_angle_rad"],
        label="$\\varphi(a)$",
        title="A-Incidence angle ($\\varphi$) in rad",
        xlabel="$\\varphi(a)$",
        ylabel="cases",
    )
    ax = fig.add_subplot(2, 2, 3)  # Initialize phi(a) log subplot
    do_incidence_angle_subplot(
        fig,
        ax,
        arec["incidence_angle_rad"],
        label="$\\varphi(a)$",
        title="A-Incidence angle ($\\varphi$) in rad (logarithmic)",
        xlabel="$\\varphi(a)$",
        ylabel="cases",
        log=True,
    )
    ax = fig.add_subplot(2, 2, 2)  # Initialize phi(b) subplot
    do_incidence_angle_subplot(
        fig,
        ax,
        brec["incidence_angle_rad"],
        label="$\\varphi(b)$",
        title="B-Incidence angle ($\\varphi$) in rad",
        xlabel="$\\varphi(b)$",
        ylabel="cases",
    )
    ax = fig.add_subplot(2, 2, 4)  # Initialize phi(a) log subplot
    do_incidence_angle_subplot(
        fig,
        ax,
        brec["incidence_angle_rad"],
        label="$\\varphi(b)$",
        title="B-Incidence angle ($\\varphi$) in rad (logarithmic)",
        xlabel="$\\varphi(b)$",
        ylabel="cases",
        log=True,
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(os.path.join(outdir, "incidence_angle_distribution_rad.png"))
    fig.clear()
    plt.close(fig)
    # Do the incidence angle plots (degrees)
    fig = init_figure()  # Initialize figure
    ax = fig.add_subplot(2, 2, 1)  # Initialize phi(a) subplot
    do_incidence_angle_subplot(
        fig,
        ax,
        arec["incidence_angle_rad"] * 180 / np.pi,
        label="$\\varphi(a)$",
        title="A-Incidence angle ($\\varphi$) in degrees",
        xlabel="$\\varphi(a)$",
        ylabel="cases",
    )
    ax = fig.add_subplot(2, 2, 3)  # Initialize phi(a) log subplot
    do_incidence_angle_subplot(
        fig,
        ax,
        arec["incidence_angle_rad"] * 180 / np.pi,
        label="$\\varphi(a)$",
        title="A-Incidence angle ($\\varphi$) in degrees (logarithmic)",
        xlabel="$\\varphi(a)$",
        ylabel="cases",
        log=True,
    )
    ax = fig.add_subplot(2, 2, 2)  # Initialize phi(b) subplot
    do_incidence_angle_subplot(
        fig,
        ax,
        brec["incidence_angle_rad"] * 180 / np.pi,
        label="$\\varphi(b)$",
        title="B-Incidence angle ($\\varphi$) in degrees",
        xlabel="$\\varphi(b)$",
        ylabel="cases",
    )
    ax = fig.add_subplot(2, 2, 4)  # Initialize phi(a) log subplot
    do_incidence_angle_subplot(
        fig,
        ax,
        brec["incidence_angle_rad"] * 180 / np.pi,
        label="$\\varphi(b)$",
        title="B-Incidence angle ($\\varphi$) in degrees (logarithmic)",
        xlabel="$\\varphi(b)$",
        ylabel="cases",
        log=True,
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(os.path.join(outdir, "incidence_angle_distribution_degrees.png"))
    fig.clear()
    plt.close(fig)


def do_y_by_x_subplot(
    fig, ax, x, y, title=None, xlabel=None, ylabel=None, color="black"
):
    if title is not None:
        ax.set_title(title, fontsize=14)
    ax.scatter(x, y, c=color, s=8)
    if xlabel is not None:
        ax.set_xlabel(xlabel, fontsize=12)
    if ylabel is not None:
        ax.set_ylabel(ylabel, fontsize=12)
    ax.tick_params(axis="both", which="both", labelsize=12)
    ax.grid("both")
    ax.set_axisbelow(True)


def do_by_incidence_angle_plots(arec, brec, outdir, emitted_power=False):
    # Validate classification calculation data
    if (
        not validate_record("incidence_angle_rad", arec, "a")
        or not validate_record("target_range_m", arec, "a")
        or not validate_record("target_area_m2", arec, "a")
        or not validate_record("radius_m", arec, "a")
        or not validate_record("bdrf", arec, "a")
        or not validate_record("cross_section", arec, "a")
        or not validate_record("emitted_power", arec, "a")
        or not validate_record("received_power", arec, "a")
        or not validate_record("incidence_angle_rad", brec, "b")
        or not validate_record("target_range_m", brec, "b")
        or not validate_record("target_area_m2", brec, "b")
        or not validate_record("radius_m", brec, "b")
        or not validate_record("bdrf", brec, "b")
        or not validate_record("cross_section", brec, "b")
        or not validate_record("emitted_power", brec, "b")
        or not validate_record("received_power", brec, "b")
    ):
        print("Cannot do by incidence angle plots")
        return
    # Plot by radians
    fname = "plot_by_incidence_angle_"
    if emitted_power:
        fname += "pe_"
    _do_by_incidence_angle_plots(
        arec["incidence_angle_rad"],
        brec["incidence_angle_rad"],
        arec,
        brec,
        outdir,
        fname=fname + "rad.png",
        unit="rad",
        emitted_power=emitted_power,
    )
    # Plot by degrees
    _do_by_incidence_angle_plots(
        arec["incidence_angle_rad"] * 180 / np.pi,
        brec["incidence_angle_rad"] * 180 / np.pi,
        arec,
        brec,
        outdir,
        fname=fname + "degrees.png",
        unit="deg",
        emitted_power=emitted_power,
    )


def _do_by_incidence_angle_plots(
    incidence_angle_a,
    incidence_angle_b,
    arec,
    brec,
    outdir,
    fname,
    unit="rad",
    emitted_power=False,
):
    # Do the "by incidence angle" plots
    fig = init_figure()  # Initialize figure
    ax = fig.add_subplot(3, 4, 1)  # Initialize target range A subplot
    do_y_by_x_subplot(
        fig,
        ax,
        incidence_angle_a,
        arec["target_range_m"],
        title="A-Target range (m)",
        xlabel=f"Incidence angle ({unit})",
        ylabel="Target range (m)",
        color="black",
    )
    ax = fig.add_subplot(3, 4, 2)  # Initialize target area A subplot
    do_y_by_x_subplot(
        fig,
        ax,
        incidence_angle_a,
        arec["target_area_m2"],
        title="A-Target area ($m^2$)",
        xlabel=f"Incidence angle ({unit})",
        ylabel="Target area ($m^2$)",
        color="tab:blue",
    )
    ax = fig.add_subplot(3, 4, 5)  # Initialize radius A subplot
    do_y_by_x_subplot(
        fig,
        ax,
        incidence_angle_a,
        arec["radius_m"],
        title="A-Radius (m)",
        xlabel=f"Incidence angle ({unit})",
        ylabel="Radius (m)",
        color="tab:red",
    )
    ax = fig.add_subplot(3, 4, 6)  # Initialize BDRF A subplot
    if emitted_power:  # Do emitted power instead
        do_y_by_x_subplot(
            fig,
            ax,
            incidence_angle_a,
            arec["emitted_power"],
            title="A-Emitted power",
            xlabel=f"Incidence angle ({unit})",
            ylabel="Emitted power",
            color="tab:green",
        )
    else:  # Do BDRF subplot as expected
        do_y_by_x_subplot(
            fig,
            ax,
            incidence_angle_a,
            arec["bdrf"],
            title="A-BDRF",
            xlabel=f"Incidence angle ({unit})",
            ylabel="BDRF",
            color="tab:green",
        )
    ax = fig.add_subplot(3, 4, 9)  # Initialize Cross-section A subplot
    do_y_by_x_subplot(
        fig,
        ax,
        incidence_angle_a,
        arec["cross_section"],
        title="A-Cross-section ($m^2$)",
        xlabel=f"Incidence angle ({unit})",
        ylabel="Cross-section ($m^2$)",
        color="tab:orange",
    )
    ax = fig.add_subplot(3, 4, 10)  # Initialize received power A subplot
    do_y_by_x_subplot(
        fig,
        ax,
        incidence_angle_a,
        arec["received_power"],
        title="A-Received power",
        xlabel=f"Incidence angle ({unit})",
        ylabel="Received power",
        color="tab:purple",
    )
    ax = fig.add_subplot(3, 4, 3)  # Initialize target range B subplot
    do_y_by_x_subplot(
        fig,
        ax,
        incidence_angle_b,
        brec["target_range_m"],
        title="B-Target range (m)",
        xlabel=f"Incidence angle ({unit})",
        ylabel="Target range (m)",
        color="black",
    )
    ax = fig.add_subplot(3, 4, 4)  # Initialize target area B subplot
    do_y_by_x_subplot(
        fig,
        ax,
        incidence_angle_b,
        brec["target_area_m2"],
        title="B-Target area ($m^2$)",
        xlabel=f"Incidence angle ({unit})",
        ylabel="Target area ($m^2$)",
        color="tab:blue",
    )
    ax = fig.add_subplot(3, 4, 7)  # Initialize radius B subplot
    do_y_by_x_subplot(
        fig,
        ax,
        incidence_angle_b,
        brec["radius_m"],
        title="B-Radius (m)",
        xlabel=f"Incidence angle ({unit})",
        ylabel="Radius (m)",
        color="tab:red",
    )
    ax = fig.add_subplot(3, 4, 8)  # Initialize BDRF B subplot
    if emitted_power:  # Do emitted power instead
        do_y_by_x_subplot(
            fig,
            ax,
            incidence_angle_b,
            brec["emitted_power"],
            title="B-Emitted power",
            xlabel=f"Incidence angle ({unit})",
            ylabel="Emitted power",
            color="tab:green",
        )
    else:  # Do BDRF subplot as expected
        do_y_by_x_subplot(
            fig,
            ax,
            incidence_angle_b,
            brec["bdrf"],
            title="B-BDRF",
            xlabel=f"Incidence angle ({unit})",
            ylabel="BDRF",
            color="tab:green",
        )
    ax = fig.add_subplot(3, 4, 11)  # Initialize Cross-section B subplot
    do_y_by_x_subplot(
        fig,
        ax,
        incidence_angle_b,
        brec["cross_section"],
        title="B-Cross-section ($m^2$)",
        xlabel=f"Incidence angle ({unit})",
        ylabel="Cross-section ($m^2$)",
        color="tab:orange",
    )
    ax = fig.add_subplot(3, 4, 12)  # Initialize received power B subplot
    do_y_by_x_subplot(
        fig,
        ax,
        incidence_angle_b,
        brec["received_power"],
        title="B-Received power",
        xlabel=f"Incidence angle ({unit})",
        ylabel="Received power",
        color="tab:purple",
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(os.path.join(outdir, fname))
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
        x,
        y,
        bins=bins,
        cmap="viridis",
        weights=100 * np.ones_like(x) / len(x),
        edgecolors="black",
    )
    fig.colorbar(hist2d[3])
    if xlabel is not None:
        ax.set_xlabel(xlabel, fontsize=14)
    if ylabel is not None:
        ax.set_ylabel(ylabel, fontsize=14)
    ax.tick_params(axis="both", which="both", labelsize=12)
    ax.grid("both")
    ax.set_axisbelow(True)


def do_subray_hit_subplot_hist(
    fig, ax, hit, x, title=None, xlabel=None, ylabel=None, bins=7, relative=False
):
    if title is not None:
        ax.set_title(title, fontsize=15)
    x_hit = x[hit]
    x_nohit = x[~hit]
    weights = (
        [
            100 * np.ones_like(x_hit) / len(x_hit),
            100 * np.ones_like(x_nohit) / len(x_nohit),
        ]
        if relative
        else None
    )
    hist = ax.hist([x_hit, x_nohit], bins=bins, label=["hit", "miss"], weights=weights)
    if xlabel is not None:
        ax.set_xlabel(xlabel, fontsize=14)
    if ylabel is not None:
        ax.set_ylabel(ylabel, fontsize=14)
    ax.tick_params(axis="both", which="both", labelsize=14)
    ax.legend(loc="upper right", fontsize=12)
    ax.grid("both")
    ax.set_axisbelow(True)


def do_subray_hit_plots(arec, brec, outdir):
    # Validate subray hit data
    if (
        not validate_record("subray_hit", arec, "a")
        or not validate_record("divergence_angle_rad", arec, "a")
        or not validate_record("subray_hit", brec, "b")
        or not validate_record("divergence_angle_rad", brec, "b")
    ):
        print("Cannot do subray hit plots")
        return

    # Do the subray hit plots
    fig = init_figure()  # Initialize figure
    # CASE A
    ax = fig.add_subplot(4, 5, 1)  # Initialize hit2Dhist on (radstep,circstep)
    # do_subray_hit_subplot_hist2d(
    #    fig, ax,
    #    arec['circle_step'][arec['subray_hit']],
    #    arec['radius_step'][arec['subray_hit']],
    #    title='Hit distribution (100%) (A)',
    # )
    # Removed because radstep and circstep are no longer exported
    ax = fig.add_subplot(4, 5, 2)  # Initialize a hist on radius step by hit
    # do_subray_hit_subplot_hist(
    #    fig, ax, arec['subray_hit'], arec['radius_step'],
    #    ylabel='Absolute'
    # )
    # Removed because radstep is no longer exported
    ax = fig.add_subplot(4, 5, 3)  # Initialize a hist on circle steps by hit
    # do_subray_hit_subplot_hist(
    #    fig, ax, arec['subray_hit'], arec['circle_steps'],
    # )
    # Removed because circstep is no longer exported
    ax = fig.add_subplot(4, 5, 4)  # Initialize a hist on circle step by hit
    # do_subray_hit_subplot_hist(
    #    fig, ax, arec['subray_hit'], arec['circle_step'],
    # )
    # Removed because circstep is no longer exported
    ax = fig.add_subplot(4, 5, 5)  # Initialize a hist on div. angle by hit
    do_subray_hit_subplot_hist(
        fig,
        ax,
        arec["subray_hit"],
        1e03 * arec["divergence_angle_rad"] * 180 / np.pi,
    )
    ax = fig.add_subplot(4, 5, 6)  # Initialize non-hit 2D hist on (rs, cs)
    # do_subray_hit_subplot_hist2d(
    #    fig, ax,
    #    arec['circle_step'][~arec['subray_hit']],
    #    arec['radius_step'][~arec['subray_hit']],
    #    title='No-hit distribution (100%)',
    #    xlabel='Circle step',
    #    ylabel='Radius step'
    # )
    # Removed because radstep and circstep are no longer exported
    ax = fig.add_subplot(4, 5, 7)  # Initialize a hist on radius step by hit
    # do_subray_hit_subplot_hist(
    #    fig, ax, arec['subray_hit'], arec['radius_step'],
    #    ylabel='Relative ($100\\%$)',
    #    relative=True,
    #    xlabel='Radius step'
    # )
    # Removed because radstep is no longer exported
    ax = fig.add_subplot(4, 5, 8)  # Initialize a hist on circle steps by hit
    # do_subray_hit_subplot_hist(
    #    fig, ax, arec['subray_hit'], arec['circle_steps'],
    #    relative=True,
    #    xlabel='Circle steps'
    # )
    # Removed because circstep is no longer exported
    ax = fig.add_subplot(4, 5, 9)  # Initialize a hist on circle step by hit
    # do_subray_hit_subplot_hist(
    #    fig, ax, arec['subray_hit'], arec['circle_step'],
    #    relative=True,
    #    xlabel='Circle step'
    # )
    # Removed because circstep is no longer exported
    ax = fig.add_subplot(4, 5, 10)  # Initialize a hist on div. angle by hit
    do_subray_hit_subplot_hist(
        fig,
        ax,
        arec["subray_hit"],
        1e03 * arec["divergence_angle_rad"] * 180 / np.pi,
        relative=True,
        xlabel="Divergence angle (deg $\\times 10^{-3}$)",
    )
    # CASE B
    ax = fig.add_subplot(4, 5, 11)  # Initialize hit2Dhist on (radstep,circstep)
    # do_subray_hit_subplot_hist2d(
    #    fig, ax,
    #    brec['circle_step'][brec['subray_hit']],
    #    brec['radius_step'][brec['subray_hit']],
    #    title='Hit distribution (100%) (B)',
    # )
    # Removed because radstep and circstep are no longer exported
    ax = fig.add_subplot(4, 5, 12)  # Initialize a hist on radius step by hit
    # do_subray_hit_subplot_hist(
    #    fig, ax, brec['subray_hit'], brec['radius_step'],
    #    ylabel='Absolute'
    # )
    # Removed because radstep is no longer exported
    ax = fig.add_subplot(4, 5, 13)  # Initialize a hist on circle steps by hit
    # do_subray_hit_subplot_hist(
    #    fig, ax, brec['subray_hit'], brec['circle_steps'],
    # )
    # Removed because circstep is no longer exported
    ax = fig.add_subplot(4, 5, 14)  # Initialize a hist on circle step by hit
    # do_subray_hit_subplot_hist(
    #    fig, ax, brec['subray_hit'], brec['circle_step'],
    # )
    # Removed because circstep is no longer exported
    ax = fig.add_subplot(4, 5, 15)  # Initialize a hist on div. angle by hit
    do_subray_hit_subplot_hist(
        fig,
        ax,
        brec["subray_hit"],
        1e03 * brec["divergence_angle_rad"] * 180 / np.pi,
    )
    ax = fig.add_subplot(4, 5, 16)  # Initialize non-hit 2D hist on (rs, cs)
    # do_subray_hit_subplot_hist2d(
    #    fig, ax,
    #    brec['circle_step'][~brec['subray_hit']],
    #    brec['radius_step'][~brec['subray_hit']],
    #    title='No-hit distribution (100%)',
    #    xlabel='Circle step',
    #    ylabel='Radius step'
    # )
    # Removed because radstep and circstep are no longer exported
    ax = fig.add_subplot(4, 5, 17)  # Initialize a hist on radius step by hit
    # do_subray_hit_subplot_hist(
    #    fig, ax, brec['subray_hit'], brec['radius_step'],
    #    ylabel='Relative ($100\\%$)',
    #    relative=True,
    #    xlabel='Radius step'
    # )
    # Removed because radstep is no longer exported
    ax = fig.add_subplot(4, 5, 18)  # Initialize a hist on circle steps by hit
    # do_subray_hit_subplot_hist(
    #    fig, ax, brec['subray_hit'], brec['circle_steps'],
    #    relative=True,
    #    xlabel='Circle steps'
    # )
    # Removed because circstep is no longer exported
    ax = fig.add_subplot(4, 5, 19)  # Initialize a hist on circle step by hit
    # do_subray_hit_subplot_hist(
    #    fig, ax, brec['subray_hit'], brec['circle_step'],
    #    relative=True,
    #    xlabel='Circle step'
    # )
    # Removed because circstep is no longer exported
    ax = fig.add_subplot(4, 5, 20)  # Initialize a hist on div. angle by hit
    do_subray_hit_subplot_hist(
        fig,
        ax,
        brec["subray_hit"],
        1e03 * brec["divergence_angle_rad"] * 180 / np.pi,
        relative=True,
        xlabel="Divergence angle (deg $\\times 10^{-3}$)",
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(os.path.join(outdir, "subray_hit.png"))
    fig.clear()
    plt.close(fig)


def do_ray_subray_plots(arec, brec, outdir):
    # Validate ray subray data
    if (
        not validate_record("ray_dir_norm", arec, "a")
        or not validate_record("subray_dir_norm", arec, "a")
        or not validate_record("ray_subray_angle_rad", arec, "a")
        or not validate_record("ray_subray_sign_check", arec, "a")
        or not validate_record("subray_tmin", arec, "a")
        or not validate_record("subray_tmax", arec, "a")
        or not validate_record("ray_dir_norm", brec, "b")
        or not validate_record("subray_dir_norm", brec, "b")
        or not validate_record("ray_subray_angle_rad", brec, "b")
        or not validate_record("ray_subray_sign_check", brec, "b")
        or not validate_record("subray_tmin", brec, "b")
        or not validate_record("subray_tmax", brec, "b")
    ):
        print("Cannot do ray-subray plots")
        return
    # Do the ray subray plots
    fig = init_figure()  # Initialize figure
    # CASE A
    ax = fig.add_subplot(3, 4, 1)  # Initialize ray norm subplot
    do_incidence_angle_subplot(
        fig, ax, arec["ray_dir_norm"], xlabel="Ray direction norm"
    )
    ax = fig.add_subplot(3, 4, 2)  # Initialize subray norm subplot
    do_incidence_angle_subplot(
        fig, ax, arec["subray_dir_norm"], xlabel="Subay direction norm"
    )
    ax = fig.add_subplot(3, 4, 5)  # Initialize ray-subray angle subplot
    do_incidence_angle_subplot(
        fig,
        ax,
        arec["ray_subray_angle_rad"] * 180 / np.pi,
        xlabel="Ray-subray angle (deg)",
    )
    ax = fig.add_subplot(3, 4, 6)  # Initialize ray-subray sign check subplot
    do_incidence_angle_subplot(
        fig, ax, arec["ray_subray_sign_check"], xlabel="Sign equality check"
    )
    ax = fig.add_subplot(3, 4, 9)  # Initialize subray tmin subplot
    do_incidence_angle_subplot(
        fig, ax, arec["subray_tmin"], xlabel="Subray $t_{\\mathrm{min}}$"
    )
    ax = fig.add_subplot(3, 4, 10)  # Initialize subray tmin subplot
    do_incidence_angle_subplot(
        fig, ax, arec["subray_tmax"], xlabel="Subray $t_{\\mathrm{max}}$"
    )
    # CASE B
    ax = fig.add_subplot(3, 4, 3)  # Initialize ray norm subplot
    do_incidence_angle_subplot(
        fig, ax, brec["ray_dir_norm"], xlabel="Ray direction norm"
    )
    ax = fig.add_subplot(3, 4, 4)  # Initialize subray norm subplot
    do_incidence_angle_subplot(
        fig, ax, brec["subray_dir_norm"], xlabel="Subay direction norm"
    )
    ax = fig.add_subplot(3, 4, 7)  # Initialize ray-subray angle subplot
    do_incidence_angle_subplot(
        fig,
        ax,
        brec["ray_subray_angle_rad"] * 180 / np.pi,
        xlabel="Ray-subray angle (deg)",
    )
    ax = fig.add_subplot(3, 4, 8)  # Initialize ray-subray sign check subplot
    do_incidence_angle_subplot(
        fig, ax, brec["ray_subray_sign_check"], xlabel="Sign equality check"
    )
    ax = fig.add_subplot(3, 4, 11)  # Initialize subray tmin subplot
    do_incidence_angle_subplot(
        fig, ax, brec["subray_tmin"], xlabel="Subray $t_{\\mathrm{min}}$"
    )
    ax = fig.add_subplot(3, 4, 12)  # Initialize subray tmin subplot
    do_incidence_angle_subplot(
        fig, ax, brec["subray_tmax"], xlabel="Subray $t_{\\mathrm{max}}$"
    )
    fig.tight_layout()
    # Save figure to file and remove it from memory
    fig.savefig(os.path.join(outdir, "ray_subray.png"))
    fig.clear()
    plt.close(fig)


def do_dir_2d_subplot(
    fig, ax, x, y, hit=None, title=None, xlabel=None, ylabel=None, legend=False
):
    if title is not None:
        ax.set_title(title, fontsize=15)
    # Plot unitary circumference
    theta = np.linspace(-np.pi, np.pi)
    ax.plot(np.cos(theta), np.sin(theta), color="black", lw=3, zorder=7)
    if hit is not None:
        ax.scatter(x[hit], y[hit], s=64, c="tab:red", zorder=6, label="hit")
        ax.scatter(x[~hit], y[~hit], s=64, c="tab:blue", zorder=5, label="miss")
        if legend:
            ax.legend(loc="upper right").set_zorder(11)
    else:
        ax.scatter(x, y, s=64, c="tab:green", zorder=6)
    if xlabel is not None:
        ax.set_xlabel(xlabel, fontsize=14)
    if ylabel is not None:
        ax.set_ylabel(ylabel, fontsize=14)
    ax.axis("equal")


def do_ray_subray_dir_plots(arec, brec, outdir):
    # Validate ray and subray direction data
    if (
        not validate_record("subray_dir_x", arec, "a")
        or not validate_record("subray_dir_y", arec, "a")
        or not validate_record("subray_dir_z", arec, "a")
        or not validate_record("ray_dir_x", arec, "a")
        or not validate_record("ray_dir_y", arec, "a")
        or not validate_record("ray_dir_z", arec, "a")
        or not validate_record("subray_dir_x", brec, "b")
        or not validate_record("subray_dir_y", brec, "b")
        or not validate_record("subray_dir_z", brec, "b")
        or not validate_record("ray_dir_x", brec, "b")
        or not validate_record("ray_dir_y", brec, "b")
        or not validate_record("ray_dir_z", brec, "b")
    ):
        print("Cannot do ray and subray direction plots")
        return
    # Do the ray subray direction plots
    fig = init_figure()  # Initialize figure
    gs = plt.GridSpec(4, 1, figure=fig)
    gs0 = mpl.gridspec.GridSpecFromSubplotSpec(1, 6, subplot_spec=gs[0], wspace=0.3)
    gs1 = mpl.gridspec.GridSpecFromSubplotSpec(1, 4, subplot_spec=gs[1])
    gs2 = mpl.gridspec.GridSpecFromSubplotSpec(1, 6, subplot_spec=gs[2], wspace=0.3)
    gs3 = mpl.gridspec.GridSpecFromSubplotSpec(1, 4, subplot_spec=gs[3])
    # CASE A
    ax = fig.add_subplot(gs0[0, 0])  # Initialize subray dir xy subplot
    subray_xynorm = np.linalg.norm(
        np.array([arec["subray_dir_x"], arec["subray_dir_y"]]), axis=0
    )
    do_dir_2d_subplot(
        fig,
        ax,
        arec["subray_dir_x"] / subray_xynorm,
        arec["subray_dir_y"] / subray_xynorm,
        hit=arec["subray_hit"],
        title="A subray (x, y)",
        xlabel="$x$",
        ylabel="$y$",
        legend=True,
    )
    ax = fig.add_subplot(gs0[0, 1])  # Initialize subray dir xz subplot
    xznorm = np.linalg.norm(
        np.array([arec["subray_dir_x"], arec["subray_dir_z"]]), axis=0
    )
    do_dir_2d_subplot(
        fig,
        ax,
        arec["subray_dir_x"] / xznorm,
        arec["subray_dir_z"] / xznorm,
        hit=arec["subray_hit"],
        title="subray (x, z)",
        xlabel="$x$",
        ylabel="$z$",
    )
    ax = fig.add_subplot(gs0[0, 2])  # Initialize subray dir yz subplot
    yznorm = np.linalg.norm(
        np.array([arec["subray_dir_y"], arec["subray_dir_z"]]), axis=0
    )
    do_dir_2d_subplot(
        fig,
        ax,
        arec["subray_dir_y"] / yznorm,
        arec["subray_dir_z"] / yznorm,
        hit=arec["subray_hit"],
        title="subray (y, z)",
        xlabel="$y$",
        ylabel="$z$",
    )
    ax = fig.add_subplot(gs0[0, 3])  # Initialize ray dir xy subplot
    ray_xynorm = np.linalg.norm(
        np.array([arec["ray_dir_x"], arec["ray_dir_y"]]), axis=0
    )
    do_dir_2d_subplot(
        fig,
        ax,
        arec["ray_dir_x"] / ray_xynorm,
        arec["ray_dir_y"] / ray_xynorm,
        title="ray (x, y)",
        xlabel="$x$",
        ylabel="$y$",
        legend=True,
    )
    ax = fig.add_subplot(gs0[0, 4])  # Initialize ray dir xz subplot
    xznorm = np.linalg.norm(np.array([arec["ray_dir_x"], arec["ray_dir_z"]]), axis=0)
    do_dir_2d_subplot(
        fig,
        ax,
        arec["ray_dir_x"] / xznorm,
        arec["ray_dir_z"] / xznorm,
        title="ray (x, z)",
        xlabel="$x$",
        ylabel="$z$",
    )
    ax = fig.add_subplot(gs0[0, 5])  # Initialize ray dir yz subplot
    yznorm = np.linalg.norm(np.array([arec["ray_dir_y"], arec["ray_dir_z"]]), axis=0)
    do_dir_2d_subplot(
        fig,
        ax,
        arec["ray_dir_y"] / yznorm,
        arec["ray_dir_z"] / yznorm,
        title="ray (y, z)",
        xlabel="$y$",
        ylabel="$z$",
    )

    ax = fig.add_subplot(gs1[0, 0])  # Initialize subray dir xy histogram
    do_subray_hit_subplot_hist(
        fig,
        ax,
        arec["subray_hit"],
        subray_xynorm,
        xlabel="Subray dir norm on (x, y)",
        ylabel="A Relative (100%)",
        relative=True,
    )
    ax = fig.add_subplot(gs1[0, 1])  # Initialize subray dir xy histogram
    do_subray_hit_subplot_hist(
        fig,
        ax,
        arec["subray_hit"],
        arec["subray_dir_z"],
        xlabel="Subray dir z",
        relative=True,
    )
    ax = fig.add_subplot(gs1[0, 2])  # Initialize subray dir xy histogram
    do_incidence_angle_subplot(
        fig,
        ax,
        ray_xynorm,
        xlabel="Ray dir norm on (x, y)",
        relative=True,
        label_fontsize=14,
    )
    ax = fig.add_subplot(gs1[0, 3])  # Initialize subray dir xy histogram
    do_incidence_angle_subplot(
        fig, ax, arec["ray_dir_z"], xlabel="Ray dir z", relative=True, label_fontsize=14
    )
    # CASE B
    ax = fig.add_subplot(gs2[0, 0])  # Initialize subray dir xy subplot
    subray_xynorm = np.linalg.norm(
        np.array([brec["subray_dir_x"], brec["subray_dir_y"]]), axis=0
    )
    do_dir_2d_subplot(
        fig,
        ax,
        brec["subray_dir_x"] / subray_xynorm,
        brec["subray_dir_y"] / subray_xynorm,
        hit=brec["subray_hit"],
        title="B subray (x, y)",
        xlabel="$x$",
        ylabel="$y$",
        legend=True,
    )
    ax = fig.add_subplot(gs2[0, 1])  # Initialize subray dir xz subplot
    xznorm = np.linalg.norm(
        np.array([brec["subray_dir_x"], brec["subray_dir_z"]]), axis=0
    )
    do_dir_2d_subplot(
        fig,
        ax,
        brec["subray_dir_x"] / xznorm,
        brec["subray_dir_z"] / xznorm,
        hit=brec["subray_hit"],
        title="subray (x, z)",
        xlabel="$x$",
        ylabel="$z$",
    )
    ax = fig.add_subplot(gs2[0, 2])  # Initialize subray dir yz subplot
    yznorm = np.linalg.norm(
        np.array([brec["subray_dir_y"], brec["subray_dir_z"]]), axis=0
    )
    do_dir_2d_subplot(
        fig,
        ax,
        brec["subray_dir_y"] / yznorm,
        brec["subray_dir_z"] / yznorm,
        hit=brec["subray_hit"],
        title="subray (y, z)",
        xlabel="$y$",
        ylabel="$z$",
    )
    ax = fig.add_subplot(gs2[0, 3])  # Initialize ray dir xy subplot
    ray_xynorm = np.linalg.norm(
        np.array([brec["ray_dir_x"], brec["ray_dir_y"]]), axis=0
    )
    do_dir_2d_subplot(
        fig,
        ax,
        brec["ray_dir_x"] / ray_xynorm,
        brec["ray_dir_y"] / ray_xynorm,
        title="ray (x, y)",
        xlabel="$x$",
        ylabel="$y$",
        legend=True,
    )
    ax = fig.add_subplot(gs2[0, 4])  # Initialize ray dir xz subplot
    xznorm = np.linalg.norm(np.array([brec["ray_dir_x"], brec["ray_dir_z"]]), axis=0)
    do_dir_2d_subplot(
        fig,
        ax,
        brec["ray_dir_x"] / xznorm,
        brec["ray_dir_z"] / xznorm,
        title="ray (x, z)",
        xlabel="$x$",
        ylabel="$z$",
    )
    ax = fig.add_subplot(gs2[0, 5])  # Initialize ray dir yz subplot
    yznorm = np.linalg.norm(np.array([brec["ray_dir_y"], brec["ray_dir_z"]]), axis=0)
    do_dir_2d_subplot(
        fig,
        ax,
        brec["ray_dir_y"] / yznorm,
        brec["ray_dir_z"] / yznorm,
        title="ray (y, z)",
        xlabel="$y$",
        ylabel="$z$",
    )
    ax = fig.add_subplot(gs3[0, 0])  # Initialize subray dir xy histogram
    do_subray_hit_subplot_hist(
        fig,
        ax,
        brec["subray_hit"],
        subray_xynorm,
        xlabel="Subray dir norm on (x, y)",
        ylabel="B Relative (100%)",
        relative=True,
    )
    ax = fig.add_subplot(gs3[0, 1])  # Initialize subray dir xy histogram
    do_subray_hit_subplot_hist(
        fig,
        ax,
        brec["subray_hit"],
        brec["subray_dir_z"],
        xlabel="Subray dir z",
        relative=True,
    )
    ax = fig.add_subplot(gs3[0, 2])  # Initialize subray dir xy histogram
    do_incidence_angle_subplot(
        fig,
        ax,
        ray_xynorm,
        xlabel="Ray dir norm on (x, y)",
        relative=True,
        label_fontsize=14,
    )
    ax = fig.add_subplot(gs3[0, 3])  # Initialize subray dir xy histogram
    do_incidence_angle_subplot(
        fig, ax, brec["ray_dir_z"], xlabel="Ray dir z", relative=True, label_fontsize=14
    )
    # Save figure to file and remove it from memory
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "ray_subray_dir.png"))
    fig.clear()
    plt.close(fig)


def do_energy_plots(arec, brec, outdir):
    # Validate energy plots
    if (
        not validate_record("incidence_angle_rad", arec, "a")
        or not validate_record("target_area_m2", arec, "a")
        or not validate_record("cross_section", arec, "a")
        or not validate_record("emitted_power", arec, "a")
        or not validate_record("received_power", arec, "a")
        or not validate_record("radius_step", arec, "a")
        or not validate_record("ray_idx", arec, "a")
        or not validate_record("incidence_angle_rad", brec, "b")
        or not validate_record("target_area_m2", brec, "b")
        or not validate_record("cross_section", brec, "b")
        or not validate_record("emitted_power", brec, "b")
        or not validate_record("received_power", brec, "b")
        or not validate_record("radius_step", brec, "b")
        or not validate_record("ray_idx", brec, "b")
    ):
        print("Cannot do energy plots")
        return
    # Remove all records with nan ray idx
    amask, bmask = ~np.isnan(arec["ray_idx"]), ~np.isnan(brec["ray_idx"])
    arec["incidence_angle_rad"] = arec["incidence_angle_rad"][amask]
    arec["target_area_m2"] = arec["target_area_m2"][amask]
    arec["cross_section"] = arec["cross_section"][amask]
    arec["emitted_power"] = arec["emitted_power"][amask]
    arec["received_power"] = arec["received_power"][amask]
    arec["radius_step"] = arec["radius_step"][amask]
    arec["ray_idx"] = arec["ray_idx"][amask]
    brec["incidence_angle_rad"] = brec["incidence_angle_rad"][bmask]
    brec["target_area_m2"] = brec["target_area_m2"][bmask]
    brec["cross_section"] = brec["cross_section"][bmask]
    brec["emitted_power"] = brec["emitted_power"][bmask]
    brec["received_power"] = brec["received_power"][bmask]
    brec["radius_step"] = brec["radius_step"][bmask]
    brec["ray_idx"] = brec["ray_idx"][bmask]
    # Find four cases equally spaced wrt incidence angle distribution
    theta_rad_a = arec["incidence_angle_rad"]
    theta_rad_b = brec["incidence_angle_rad"]
    theta_rad = np.concatenate([theta_rad_a, theta_rad_b])
    theta_min, theta_max = np.min(theta_rad), np.max(theta_rad)
    n_cases = 4
    theta_linspace = np.linspace(theta_min, theta_max, n_cases)
    ray_indices_a = []
    ray_indices_b = []
    for theta_target in theta_linspace:
        theta_index_a = np.argmin(np.abs(theta_rad_a - theta_target))
        ray_idx_a = int(arec["ray_idx"][theta_index_a])
        ray_indices_a.append(ray_idx_a)
        theta_index_b = np.argmin(np.abs(theta_rad_b - theta_target))
        ray_idx_b = int(brec["ray_idx"][theta_index_b])
        ray_indices_b.append(ray_idx_b)
    # For each case generate a dataset with all the subrays
    ray_idx_a, ray_idx_b = arec["ray_idx"], brec["ray_idx"]
    energy_dataset_a, energy_dataset_b = [], []
    for i in range(n_cases):
        mask_a = ray_idx_a == ray_indices_a[i]  # Records in a for given ray
        mask_b = ray_idx_b == ray_indices_b[i]  # Records in b for given ray
        energy_dataset_a.append(
            {
                "incidence_angle_rad": theta_rad_a[mask_a],
                "target_area_m2": arec["target_area_m2"][mask_a],
                "cross_section": arec["cross_section"][mask_a],
                "emitted_power": arec["emitted_power"][mask_a],
                "received_power": arec["received_power"][mask_a],
                "radius_step": arec["radius_step"][mask_a],
                "ray_idx": arec["ray_idx"][mask_a],
            }
        )
        energy_dataset_b.append(
            {
                "incidence_angle_rad": theta_rad_b[mask_b],
                "target_area_m2": brec["target_area_m2"][mask_b],
                "cross_section": brec["cross_section"][mask_b],
                "emitted_power": brec["emitted_power"][mask_b],
                "received_power": brec["received_power"][mask_b],
                "radius_step": brec["radius_step"][mask_b],
                "ray_idx": brec["ray_idx"][mask_b],
            }
        )
    # Do the energy plots
    _do_energy_plots(energy_dataset_a, energy_dataset_b, outdir)


def _do_energy_plots(eda, edb, outdir):
    # Prepare figures
    n_cases = 2 * len(eda)
    nrows = int(np.sqrt(n_cases))
    ncols = int(np.ceil(n_cases / nrows))
    fig_data = [
        {"key": "emitted_power", "full_name": "Emitted power", "name": "$P_e$"},
        {"key": "target_area_m2", "full_name": "Target area", "name": "$A$"},
        {"key": "cross_section", "full_name": "Cross-section", "name": "$\\sigma$"},
        {"key": "received_power", "full_name": "Received power", "name": "$P_r$"},
    ]
    for fig_record in fig_data:
        # Build figure
        fig = init_figure(figsize=(16, 10))  # Initialize figure
        # Do the subplots
        for i in range(n_cases // 2):
            ax = fig.add_subplot(nrows, ncols, i + 1)
            do_energy_subplots(
                fig,
                ax,
                eda[i],
                "A",
                fig_record["key"],
                fig_record["full_name"],
                fig_record["name"],
            )
            ax = fig.add_subplot(nrows, ncols, i + 1 + n_cases // 2)
            do_energy_subplots(
                fig,
                ax,
                edb[i],
                "B",
                fig_record["key"],
                fig_record["full_name"],
                fig_record["name"],
            )
        # Post-process figure
        fig.tight_layout()
        # Save figure to file and remove it from memory
        fig.savefig(os.path.join(outdir, f'energy_plots_{fig_record["key"]}.png'))
        fig.clear()
        plt.close(fig)


def do_energy_subplots(fig, ax, edi, case_letter, key, full_name, name):
    # Extract values of interest
    rstep = edi["radius_step"]
    n_subrays = len(rstep)
    rstep_uniq = np.unique(rstep)
    # Group by radius step
    x = edi[key]
    x_by_rs = [x[rstep == rstepk] for rstepk in rstep_uniq]
    # Do the plot
    x_by_rs_sum = 0
    for j, rstepk in enumerate(rstep_uniq):
        x_by_rsj = x_by_rs[j]
        x_by_rsj_sum = np.sum(x_by_rsj)
        x_by_rs_sum += x_by_rsj_sum
        ax.bar(
            rstepk,
            x_by_rsj_sum,
            color="gray",
            align="center",
            width=0.8,
            label="sum" if j == 0 else None,
        )
        ax.bar(
            rstepk,
            x_by_rsj[0],
            color="tab:red",
            align="center",
            width=0.8,
            label="subray" if j == 0 else None,
        )
    ax.legend(loc="best")
    ax.set_xlabel("Radius step", fontsize=14)
    ax.set_ylabel(f"{full_name} ({name})", fontsize=14)
    ax.set_title(
        f"{case_letter}) {name} $ = $ {x_by_rs_sum:.3g} ({n_subrays} subrays)\n"
        f'$\\theta = {(edi["incidence_angle_rad"][0]*180/np.pi):.3f}$ deg'
    )


# ---   M A I N   --- #
# ------------------- #
if __name__ == "__main__":
    # Prepare plotter
    args = parse_args()
    sep = ","
    # Read A records
    print('Reading A-records from "{path}" ...'.format(path=args["dira_path"]))
    start = time.perf_counter()
    arec = read_records(args["dira_path"], sep=sep)
    end = time.perf_counter()
    print("Read A-records in {t} seconds".format(t=end - start))
    # Read B records
    print('Reading B-records from "{path}" ...'.format(path=args["dirb_path"]))
    start = time.perf_counter()
    brec = read_records(args["dirb_path"], sep=sep)
    end = time.perf_counter()
    print("Read B-records in {t} seconds".format(t=end - start))
    # Plot records
    print('Generating plots at "{path}" ...'.format(path=args["dirout_path"]))
    start = time.perf_counter()
    plot_records(arec, brec, args["dirout_path"])
    end = time.perf_counter()
    print("Generated plots in {t} seconds".format(t=end - start))
