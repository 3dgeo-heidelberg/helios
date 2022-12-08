# ✈ HELIOS ++
[![DOI](https://zenodo.org/badge/331344393.svg)](https://zenodo.org/badge/latestdoi/331344393) [![Issues](https://img.shields.io/github/issues/3dgeo-heidelberg/helios)](https://github.com/3dgeo-heidelberg/helios/issues) [![License](https://img.shields.io/badge/license-GPLv3%2C%20LGPLv3-blue)](https://github.com/3dgeo-heidelberg/helios/blob/main/LICENSE.md) [![Activity](https://img.shields.io/github/commit-activity/m/3dgeo-heidelberg/helios)](https://github.com/3dgeo-heidelberg/helios/commits) [![Downloads](https://img.shields.io/github/downloads/3dgeo-heidelberg/helios/total)](https://github.com/3dgeo-heidelberg/helios/releases)
[![Build](https://github.com/3dgeo-heidelberg/helios/actions/workflows/cmake.yml/badge.svg)](https://github.com/3dgeo-heidelberg/helios/actions)

> Heidelberg LiDAR Operations Simulator ++

![Logo](h++.png)

HELIOS++ is a general-purpose software package for simulation of terrestrial, mobile and airborne laser scanning surveys written in C++11. 
It is developed and maintained by the [3DGeo Research Group](https://uni-heidelberg.de/3dgeo) at Heidelberg University.

## 💻 Download

Precompiled versions for Windows and Ubuntu (other Debian might also work, but you need to install dependencies manually) are available under [releases](https://github.com/3dgeo-heidelberg/helios/releases).

## ℹ Documentation

As a starting point, please consult the [wiki](https://github.com/3dgeo-heidelberg/helios/wiki/First-steps). 
We suggest you take the "first steps" tour to get to know the core concepts of the software.

Official website: https://uni-heidelberg.de/helios

For scientific and collaboration inquiries please contact the HELIOS++ team at helios@uni-heidelberg.de

We have also published two papers on HELIOS++. If you use HELIOS++ in a scientific context, please cite one of the following:

- General description of the framework:
> Winiwarter, L., Esmorís Pena, A., Weiser, H., Anders, K., Martínez Sanchez, J., Searle, M., Höfle, B. (2022): **Virtual laser scanning with HELIOS++: A novel take on ray tracing-based simulation of topographic full-waveform 3D laser scanning**. _Remote Sensing of Environment_, 269, doi:10.1016/j.rse.2021.112772
- High performance computing:
> Esmorís, A. M., Yermo, M., Weiser, H., Winiwarter, L., Höfle, B., Rivera, F. F. (2022): **Virtual LiDAR Simulation as a High Performance Computing Challenge: Toward HPC HELIOS++**. _IEEE Access_, 10, doi:10.1109/ACCESS.2022.3211072

BibTeX:
```
article{heliosPlusPlus,
title = {Virtual laser scanning with HELIOS++: A novel take on ray tracing-based simulation of topographic full-waveform 3D laser scanning},
journal = {Remote Sensing of Environment},
year = {2022},
volume = {269},
issn = {0034-4257},
doi = {https://doi.org/10.1016/j.rse.2021.112772},
url = {https://www.sciencedirect.com/science/article/pii/S0034425721004922},
author = {Lukas Winiwarter and Alberto Manuel {Esmorís Pena} and Hannah Weiser and Katharina Anders and Jorge {Martínez Sánchez} and Mark Searle and Bernhard Höfle},
keywords = {Software, LiDAR simulation, Point cloud, Data generation, Voxel, Vegetation modelling, Diffuse media}
} 
```

```
@Article{Esmoris2022_HPC-HELIOS,
  author={Esmorís, Alberto M. and Yermo, Miguel and Weiser, Hannah and Winiwarter, Lukas and Höfle, Bernhard and Rivera, Francisco F.},
  journal={IEEE Access},
  title={Virtual LiDAR Simulation as a High Performance Computing Challenge: Toward HPC HELIOS++},
  year={2022},
  volume={10},
  issn = {2169-3536},
  pages={105052--105073},
  doi={https://doi.org/10.1109/ACCESS.2022.3211072},
  url={https://ieeexplore.ieee.org/document/9906068}
}
```

## 🎮 Examples

| [![Example 1](img/example1_thumbnail.png)](https://nbviewer.org/github/3dgeo-heidelberg/helios/blob/dev/example_notebooks/1-getting-started.ipynb)  | [![Example 2](img/example2_thumbnail.png)](https://nbviewer.org/github/3dgeo-heidelberg/helios/blob/dev/example_notebooks/2-the-survey.ipynb)        |
|-----------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------|
| [![Example 3](img/example3_thumbnail.png)](https://nbviewer.org/github/3dgeo-heidelberg/helios/blob/dev/example_notebooks/3_arbaro_TLS_demo.ipynb) | [![Example A](img/exampleA_thumbnail.png)](https://nbviewer.org/github/3dgeo-heidelberg/helios/blob/dev/example_notebooks/arboretum_notebook.ipynb)  |


## ⌨ Usage

HELIOS++ can be invoked with following syntax:

```
helios --help
    Show the help for helios++ usage

helios --test
    Perform necessary tests to check everything works as expected
    
helios --version
    Show the HELIOS++ version details

helios <survey_file_path> [OPTIONAL ARGUMENTS]
    Perform requested simulation.

    NOTICE specifying the path to the survey specification file is mandatory

    Available general OPTIONAL ARGUMENTS are:
        --assets <directory_path>
            Specify the path to assets directory
        --output <directory_path>
            Specify the path to output directory
        --splitByChannel
            Enable the one-file-per-device writing mode when using a
            multi-channel scanner.
        --writeWaveform
            Specify the full waveform must be written
        --calcEchowidth
            Specify the full waveform must be fitted
        --fullwaveNoise
            Enable random noise at full waveform computation
        --fixedIncidenceAngle
            Sets incidence angle to exactly 1.0 for all intersections
        --seed <seed>
            Specify the seed to be used for randomness generation.
            The seed can be an integer number, a decimal number or a timestamp
            string with format "YYYY-mm-DD HH:MM:SS"
        --gpsStartTime <string>
            Specify the GPS start time. By default it is an empty string "",
            which means using current system time.
            It can be given as both, a posix timestamp as string or a datetime
            string with format "YYYY-MM-DD hh:mm:ss"
        --lasOutput
            Specify the output point cloud must be generated using LAS format
        --zipOutput
            Specify the output point cloud and fullwave must be zipped
        --lasScale
            Specify the scale factor used to generate LAS output
        --parallelization <integer>
            Specify the parallelization strategy. Where 0 leads to a simple
            static/dynamic chunk based strategy and 1 leads to a warehouse
            based strategy
        -j OR --njobs OR --nthreads <integer>
            Specify the number of simultaneous threads to be used to compute
            the simulation
            If it is not specified or it is specified as 0, then all available
            threads will be used to compute the simulation
        --chunkSize <integer>
            Specify the chunk size. If it is positive, it will be used as a
            fixed size but if it is negative the absolute value will be used
            as starting size of a dynamic chunk-size based strategy.
        --warehouseFactor <integer>
            The number of tasks in the warehouse would be k times the number
            of workers. Greater factor implies less probability of idle cores
            at expenses of greater memory consumption.
        --rebuildScene
            Force scene rebuild even when a previosly built scene is available
        --kdt <integer>
            Specify the type of KDTree to be built for the scene.
            Using 1 leads to the simple KDTree based on median balancing,
            2 to the SAH based KDTree, 3 for the SAH with best axis criteria
            and 4 (the default) to the fast approximation of SAH
        --kdtJobs <integer>
            Specify the number of threads to be used for building the KDTree.
            Using 1 forces sequential building, 0 as many threads as available
            cores and n>1 implies using exactly n threads.
            Using more cores than required might degrade performance due to
            overhead.
        --kdtGeomJobs <integer>
            Specify the number of threads to be used for upper levels of
            KDTree building.
            By default it is 0, which means as many jobs as --kdtJobs
            Using 1, means no geometry-level parallelization will be used when
            building the KDTree
            Using >1, means exactly n threads will be used at geometry-level
            KDTree building
        --sahNodes <integer>
            Either how many nodes must be used by the Surface Area Heuristic
            or the number of bins for the fast approximation of SAH
        --disablePlatformNoise
            Disable platform noise, no matter what is specified on XML files
        --disableLegNoise
            Disable leg noise, no matter what is specified on XML files

    Available logging verbosity OPTIONAL ARGUMENTS are:
        --silent
            Nothing will be reported
        -q OR --quiet
            Only errors will be reported
        -vt
            Time and errors will be reported
        -v
            Errors, information and warnings will be reported
        -vv OR -v2
            Everything will be reported
        IF NONE IS SPECIFIED
            Errors and information will be reported by default

    Available logging output mode OPTIONAL ARGUMENTS are:
        --logFile
            Reports will be emitted through standard output and output file
        --logFileOnly
            Reports will be emitted through output file only
        IF NONE IS SPECIFIED
            Reports will be emitted through standard output only

    Unzip compressed output:
        --unzip <input_path> <output_path>
            When helios++ is executed with --zipOutput flag, output files are
            compressed. They can be decompressed using --unzip.
            The path to a readable helios++ compressed output file must be
            given through input path.
            The path to a writable file/location must be given through
            output path.
```

The demo simulation can be executed as follows:

**LINUX**
```
run/helios data/surveys/demo/tls_arbaro_demo.xml
```

**WINDOWS**
```
run\helios.exe data/surveys/demo/tls_arbaro_demo.xml
```


## 🛠 Building from source

Build instructions for advanced users and developers are available [here](BUILDME.md).

## 🐍 Running pyhelios

For running pyhelios, we suggest setting up a seperate [conda](https://docs.conda.io/en/latest/miniconda.html) environment. After downloading one of the [releases](https://github.com/3dgeo-heidelberg/helios/releases) or building from source, run 
```
conda env create -f conda-environment.yml
```
in the `base` environment of your conda installation, while you are in the HELIOS++ root directory. Then run
```
conda activate pyhelios_env
```
to activate the environment and 
```
python pyhelios_demo\helios.py data\surveys\toyblocks\als_toyblocks.xml
```
to run a demo survey including visualisation.

## :earth_africa: QGIS Plugin

Our QGIS Plugin AEOS, embeds HELIOS++ into one of the most widely used GIS applications. It enables the creation of HELIOS++ surveys using QGIS vector and raster layers and the subsequent execution of the surveys, with direct availability of the results in the form of a QGIS point cloud layer. Crucially, it allows for instant visualisation of both the input and output of a HELIOS++ simulation within a familiar user interface, thereby greatly improving ease of use. In Greek mythology, Aeos is the name of one of the four horses that pulls Helios' fiery chariot accross the sky. Feel free to download AEOS from its own [GitHub repo](https://github.com/3dgeo-heidelberg/aeos) and add it to your arsenal of QGIS plugins now!

## 📜 License

See [LICENSE.md](LICENSE.md)
