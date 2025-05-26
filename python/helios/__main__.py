import click
from click_option_group import optgroup, MutuallyExclusiveOptionGroup
from helios.settings import ExecutionSettings, OutputSettings, LogVerbosity
from helios.survey import Survey
from helios.utils import add_asset_directory, set_rng_seed


@click.command()
@click.argument("survey_file_path", required=True)
@optgroup.group("Input")
@optgroup.option(
    "--assets",
    "-a",
    type=click.Path(exists=True, file_okay=False),
    multiple=True,
    help=(
        "Specify the path(s) to assets/data directory. To specify multiple "
        "paths, duplicate the argument,e.g. --assets path1 --assets path2. "
        "By default: './assets/' and the Python package installation directory."
    ),
)
@optgroup.group("Output")
@optgroup.option(
    "--output",
    "-o",
    type=click.Path(file_okay=False),
    default="./output",
    show_default=True,
    help="Specify the path to output directory",
)
@optgroup.option(
    "--splitByChannel",
    is_flag=True,
    help=(
        "Use this flag to enable the one-file-per-device writing mode when "
        "using a multi-channel scanner. "
        "By default one-file-for-all writing is enabled "
    ),
)
@optgroup.option(
    "--writeWaveform",
    is_flag=True,
    help=(
        "Use this flag to enable full waveform writing By default waveform "
        "is NOT written to output file"
    ),
)
@optgroup.option(
    "--writePulse",
    is_flag=True,
    help=(
        "Use this flag to enable pulse-wise data writing. By default "
        "pulse-wise data is NOT written to output file"
    ),
)
@optgroup.option(
    "--fullwaveNoise",
    is_flag=True,
    help=(
        "Use this flag to add noise when computing full waveform. "
        "By default: full waveform noise is disabled"
    ),
)
@optgroup.option(
    "--format",
    default="npy",
    show_default=True,
    help="Output format, can be las, laz, xyz, npy, or laspy.",
)
@optgroup.option(
    "--lasOutput",
    is_flag=True,
    help="Use this flag to generate the output point cloud in LAS format (v 1.4)",
)
@optgroup.option(
    "--las10", is_flag=True, help="Use this flag to write in LAS format (v 1.0)"
)
@optgroup.option(
    "--lasScale",
    type=click.FLOAT,
    default=0.0001,
    show_default=True,
    help="Specify the decimal scale factor for LAS output",
)
@optgroup.option(
    "--zipOutput", is_flag=True, help=("Use this flag to generate compressed output")
)
@optgroup.group("Execution")
@optgroup.option(
    "--calcEchowidth",
    is_flag=True,
    help=(
        "Use this flag to enable full waveform fitting. "
        "By default the full waveform is NOT fitted"
    ),
)
@optgroup.option(
    "--fixedIncidenceAngle",
    is_flag=True,
    help=(
        "Use this flag to use fixed incidence angle. Fixed incidence angle of "
        "exactly 0.0 will be considered for all intersection"
    ),
)
@optgroup.option(
    "--seed",
    type=click.INT,
    help=(
        "Specify the seed for randomness generation. It must be an intenger"
        "By default: a random seed is generated."
    ),
)
@optgroup.option(
    "--gpsStartTime",
    type=click.STRING,
    help=(
        "Specify a fixed start time for GPS. It can be either a posix timestamp "
        "or a 'YYYY-MM-DD hh:mm:ss+00:00' date time string, including "
        "the timezone. By default: The current system time is used."
    ),
)
@optgroup.option(
    "--parallelization",
    type=click.INT,
    default=0,
    show_default=True,
    help=(
        "Specify the parallelization strategy. 0 for a static/dynamic chunk "
        "based parallelization and 1 for a warehouse based one."
    ),
)
@optgroup.option(
    "--num-threads",
    "-j",
    "--njobs",
    type=click.INT,
    show_default=True,
    help=(
        "Specify the number of threads to be used to compute the simulation. "
        "By default: all available threads are used."
    ),
)
@optgroup.option(
    "--chunkSize",
    type=click.INT,
    default=32,
    show_default=True,
    help=(
        "Specify the chunk size to be used for parallel computing. If a "
        "negative number is given, then its absolute value is used as starting "
        "size of the dynamic chunk-size strategy. Positive numbers specify "
        "the size for a static chunk-size strategy"
    ),
)
@optgroup.option(
    "--warehouseFactor",
    type=click.INT,
    default=4,
    show_default=True,
    help=(
        "Specify the warehouse factor. The number of tasks in the warehouse "
        "would be k times the number of workers. The greater the factor, theless "
        "the probability of idle cores but the greater the memory consumption."
    ),
)
@optgroup.option(
    "--rebuildScene",
    is_flag=True,
    help=("Force scene rebuild even when a previously built scene is available"),
)
@optgroup.option(
    "--noSceneWriting",
    is_flag=True,
    help=(
        "If a scene is created during asset loading, it will be written by "
        "default. Enabling this flag will prevent this writing."
    ),
)
@optgroup.option(
    "--disablePlatformNoise",
    is_flag=True,
    help=(
        "Disable platform noise, no matter what is specified on XML files. "
        "By default: XML specifications are considered "
    ),
)
@optgroup.option(
    "--disableLegNoise",
    is_flag=True,
    help=(
        "Disable leg noise, no matter what is specified on XML files. "
        "By default: XML specifications are considered "
    ),
)
@optgroup.option(
    "--dryrun",
    is_flag=True,
    help="Do not run any calculations. Only validates inputs and exits.",
)
@optgroup.group("KDTree")
@optgroup.option(
    "--kdt",
    type=click.INT,
    default=4,
    show_default=True,
    help=(
        "Specify the type of KDTree to be built for for the scene. Using 1 is "
        "for the simple KDTree based on median balancing, 2 for the SAH "
        "based KDTree, 3 for the SAH with best axis based KDTree and "
        "4 (default) for a fast SAH approximation."
    ),
)
@optgroup.option(
    "--kdtJobs",
    type=click.INT,
    help=(
        "Specify the number of threads to be used for building the KDTree. "
        "If 1, then the KDTree will be built in a sequential fashion. "
        "If >1, then the KDTree will be built in a parallel fashion. By "
        "default, the KDTree will be built using as many threads as available."
    ),
)
@optgroup.option(
    "--kdtGeomJobs",
    type=click.INT,
    help=(
        "Specify the number of threads to be used for bu upper nodes of the "
        "KDTree (geometry-level parallelization). If 1, then there is no "
        "geometry-level parallelization. If >1, then geometry-level "
        "parallelization uses as many threads a . By default "
        "geometry-level parallelization uses as many threads as node-level."
    ),
)
@optgroup.option(
    "--sahNodes",
    type=click.INT,
    default=32,
    show_default=True,
    help=(
        "Specify how many nodes must be used by the Surface Area Heuristic "
        "when building a SAH based KDTree. For the SAH KDTree it is "
        "recommended to be 21. More nodes lead to a best search process to "
        "find split position, at the expenses of a greater computational "
        "cost. When using a fast SAH approximation it is recommended to set "
        "this to 32 (default). "
    ),
)
@optgroup.group("Logging", cls=MutuallyExclusiveOptionGroup)
@optgroup.option(
    "--logFile",
    is_flag=True,
    help=(
        "Logging will be outputted to a file, not only to standard output. "
        "By default: logging will be written to standard output."
    ),
)
@optgroup.option(
    "--logFileOnly",
    is_flag=True,
    help=(
        "Logging will be outputted ONLY to a file. "
        "By default: logging will be outputted to standard output."
    ),
)
@optgroup.group("Verbosity", cls=MutuallyExclusiveOptionGroup)
@optgroup.option(
    "--verbose",
    "-v",
    count=True,
    help=(
        "Increase the verbosity level to include information. Can be repeated "
        "once to report all messages. Default: report errors and warnings."
    ),
)
@optgroup.option(
    "--silent",
    "-s",
    is_flag=True,
    help=("Disable all logging output."),
)
@optgroup.option("--quiet", "-q", is_flag=True, help="Only errors are reported.")
@optgroup.option("--vt", is_flag=True, help="Report time and errors.")
@click.version_option()
def cli(**kw):

    # TODO:
    # The following parameters are defined but not yet implemented:
    # * disableLegNoise
    # * disablePlatformNoise
    # * noSceneWriting
    # * rebuildScene
    # * fixedIncidenceAngle
    # * calcEchowidth
    # * zipOutput
    # * las10
    # * fullwaveNoise

    for asset in kw["assets"]:
        add_asset_directory(asset)

    if seed := kw.get("seed") is not None:
        set_rng_seed(seed)
    else:
        set_rng_seed()

    verbosity = LogVerbosity.DEFAULT
    if kw.get("verbose") == 1:
        verbosity = LogVerbosity.VERBOSE
    elif kw.get("verbose") == 2:
        verbosity = LogVerbosity.VERY_VERBOSE
    elif kw.get("quiet"):
        verbosity = LogVerbosity.QUIET
    elif kw.get("vt"):
        verbosity = LogVerbosity.TIME
    elif kw.get("silent"):
        verbosity = LogVerbosity.SILENT

    output_settings = OutputSettings()
    execution_settings = ExecutionSettings()

    execution_settings.parallelization = kw.get("parallelization")
    execution_settings.num_threads = kw.get("num_threads")
    execution_settings.chunk_size = kw.get("chunksize")
    execution_settings.warehouse_factor = kw.get("warehousefactor")
    execution_settings.log_file = kw.get("logfile")
    execution_settings.log_file_only = kw.get("logfileonly")
    execution_settings.verbosity = verbosity
    execution_settings.factory_type = kw.get("kdt")
    execution_settings.kdt_num_threads = kw.get("kdtjobs")
    execution_settings.kdt_geom_num_threads = kw.get("kdtgeomjobs")
    execution_settings.sah_nodes = kw.get("sahnodes")

    output_settings.format = kw.get("format")
    output_settings.split_by_channel = kw.get("splitbychannel")
    output_settings.output_dir = kw.get("output")
    output_settings.write_waveform = kw.get("writewaveform")
    output_settings.write_pulse = kw.get("writepulse")
    output_settings.las_scale = kw.get("lasscale")

    survey = Survey.from_xml(kw.get("survey_file_path"))
    if gps := kw.get("gpsstarttime"):
        survey.gps_time = gps

    if kw.get("dryrun"):
        return
    survey.run(execution_settings=execution_settings, output_settings=output_settings)


if __name__ == "__main__":
    cli()
