# HELIOS++ DEBUG

HELIOS++ is a complex software that is composed of multiple modules. It is also
a software that is experimenting a continuous growth. Thus, a set of useful
debugging tools have been developed to assist the developers and advanced users
to understand how it works and to detect bugs easily.

<span style="color: red;"><b>WARNING!</b></span> *The debugging tools are openly
provided to anyone according to the open access and free software nature of
HELIOS++. However, the development team is not providing support for users
who are using these tools. We are neither developing debugging tools on demand.
Any user is welcome to use our debugging tools. But keep in mind that you are
doing it on your own.*









## Demos

There are different demos that can be run when HELIOS++ is compiled with [Point
Cloud Library (PCL)](https://pointclouds.org/) bindings enabled. The following
compilation flag is enough to achieve this:

```
-DPCL_BINDING=1
```

Alternatively, the PCL bindings can be disabled by means of specifying:

```
-DPCL_BINDING=0
```

However, explicitly disabling the PCL bindings should not be necessary since it
is the default option.


#### Simple demo

The simple demo is a standalone demo that is meant to be used to check that the
demos module was correctly loaded and that is working properly. It can be run
with the following execution command:

```
./helios --demo simple_primitives
```

If it is run successfully, then the next scene should be visualized:

<img src="https://drive.google.com/uc?export=view&id=1OXPxDr_QSAOqrcgYYIfopkc1oQB2kEid" alt="Simple primitives demo" width="400" height="300"/>


#### Scene demo

The scene demo can be used to visualize a scene as composed by HELIOS++. It
has the following syntax:

```
./helios --demo dynamic_scene --demoSurvey <path to survey> --demoAssets <path to assets>
```

The `--demo dynamic_scene` argument specifies the type of demo. For this case,
the *dynamic_scene* demo supports both static and dynamic scenes.

The `--demoSurvey <path to survey>` argument must be used to specify the survey
which scene has to be visualized.

The `--demoAssets <path to assets>` arguemtn must be used to specify the path
to the assets directory.


As an example, the following command can be used to visualize the moving TLS
toyblocks scene:
```
./helios --demo dynamic_scene --demoSurvey data/surveys/toyblocks/moving_tls_toyblocks.xml --demoAssets assets/
```

If it is run successfully, then the next scene should be visualized:

<img src="https://drive.google.com/uc?export=view&id=15wXlsjcv7-PP9ziTcSDaNlsDhorFIfJt" alt="Simple primitives demo" width="400" height="300"/>

It is worth to mention that pressing the key `h` will show a help message
through the terminal/console explaining the behavior associated to different
keys. The more interesting ones are listed below:

- `w` Visualizes the primitives as a wireframe
- `s` Visualizes the primitives as a surface
- `p` Visualizes the primitives as points
- `n` Toggles the rendering of normal vectors for each primitive

The mouse can be intuitively used to control the camera.


#### Ray casting demo

The ray casting demo can be used to visualize the ray casting process of a
given survey. It renders the ray casting process thoroughly. Sometimes, it is
interesting to observe each scan line in detail. For these cases, configuring
a high pulse frequency for the survey will do the trick. Other times, it is
interesting to have a global perspective on the ray casting process. For these
cases, configuring a low pulse frequency for the survey will do the trick.

The ray casting demo has the following syntax:

```
./helios --demo raycasting --demoSurvey <path to survey> --demoAssets <path to assets>
```

The `--demoSurvey` and `--demoAssets` arguments work like those described for
the scene demo case.

As an example, the following command can be used to visualize the ray casting
of the TLS arbaro demo:

```
./helios --demo raycasting --demoSurvey data/surveys/demo/tls_arbaro_demo.xml --demoAssets assets/
```

If it is run successfully, then the next ray casting process should be
visualized:

<img src="https://drive.google.com/uc?export=view&id=1du4Bfj6bjECy1fktbAsy8OPo_XQ1CgJH" alt="Simple primitives demo" width="400" height="300"/>

In this demo, the red sphere represents the position of the scanner and the
line that starts at the sphere represents the ray. The color of the ray is
red when the pulse is effectively emitted and blue when it is not.








## Data analytics

While standard debugging and profiling tools are quite useful, there are
certain bugs and situations when they are not the adequate tool. For this
purpose, we have started working in a compilation mode that injects code into
HELIOS++ to extract useful data that can be analyzed to compare, visualize, and
quantify different scenes, platforms, and scanners. To enable these code
injections, the following compilation flag must be specified:

```
-DDATA_ANALYTICS=1
```

All the data analytics utilities detailed below require the previous flag to be
enabled. Also, please note that the code injections might cause HELIOS++ to run
abnormally and significantly slower. They are not meant to be used for
normal executions.


#### JSON Report

When running HELIOS++ in data analytics mode, a file named `helios_state.json`
will be written. This file uses the JSON format to store a detailed
representation of the different variables defining a simulation. Note that
this file will be overwritten each time that HELIOS++ is run again. Thus,
remember to move it to a file with different name in case you want to preserve
it.

Any pair of JSON report files can be compared with the python script at
`scripts/debug/hda_diff_report.py`. Its syntax is simple:

```
python3 scripts/debug/hda_diff_report.py <First JSON report> <Second JSON report>
```

The resulting output warns about each difference between the two JSON reports
and shows the total number of differences.


#### Recording simulation steps

When running HELIOS++ in data analytics mode, a directory named
`helios_sim_records` will contain CSV files with different variables over
simulation steps/time. Note that this directory and all the files inside it
will be overwritten each time that HELIOS++ is run again. Thus, remember to
move them to a directory with different name in case you want to preserve them.

Any pair of directories containing simulation records can be graphically
compared with the python script at `scripts/debug/hda_simstep_plotter.py`.
Its syntax is simple:

```
python3 scripts/debug/hda_simstep_plotter <First dir> <Second dir> <Output dir>
```

The generated plots will be exported in the given output directory. They
compare the behavior of the different components (such as the beam,
the deflector, the platform, the platform's mount, the scanner,
the scanner's head, ...) involved in both simulations as shown in the plot
below.

<img src="https://drive.google.com/uc?export=view&id=1yfHzc1JiuifervHjgsZ6Avxm_kxhTR8x" alt="Platform mount attitude" width="600" height="400"/>










## Performance measurement

Sometimes it is interesting to analyze HELIOS++ performance. Whether we are
interested in studying its multithreading scalability or the impact of
different configurations, it is necessary to collect and process the execution
time for each case. This can be a tedious work. Fortunately, we have developed a
script to do this for us.

The first step is to execute all the simulations redirecting the output to a
file and using the "time and errors only" verbosity level `-vt`.  All the
output files must be placed into the same folder and named according to the
following pattern:

```
*_KDT#_SAH#_PS#_CS#_WF#_SC#_BC#.log
```

The `*` can be replaced by any text that is useful for us to identify our
battery of executions. The `#` must be replaced by the adequate configuration
of its associated argument. For instance, when using the Fast SAH KDTree with
32 SAH loss nodes, a warehouse parallelization strategy with chunks of 32
tasks, a warehouse factor of 4, 8 threads to run the simulation, and 5 threads
to build the KDTree, then a valid filename would be:

```
MyExec_KDT4_SAH32_PS1_CS32_WF4_SC8_BC5.log
```

The values for each argument match exactly those used by HELIOS++, which can be
found both in the help message (`--help`) and in the README file.

Once we have gathered all the output log files containing the performance
measurements, it is time to use the bash script at
`scripts/debug/logs_to_plots.sh`. This script uses the `plot_log_data.py`
python script in the same folder to generate a few plots representing the
performances. Besides the plots, a text summary is also printed through the
standard output. It compares the different cases and indicates the best one
for each strategy. The input arguments for the bash script are (1) the path to
the directory where the log files are stored, (2) the path where a CSV output
file containing the times for each case must be written, and (3) the path to
the directory where the different plots must be exported:

```
./logs_to_plots.sh <Path to logs directory> <Path to output csv> <Path to output plots directory>
```

It can be the case that some errors about `module load` commands are shown.
They are not critical for the script and can be safely ignored. They are
in the script to handle the loading of necessary modules in the context of the
[CESGA](https://www.cesga.es/) supercomputing infrastructure.


