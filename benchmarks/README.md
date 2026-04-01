# Benchmarking setup

This directory provides several benchmarks for performance analyses using the [Google Benchmark](https://github.com/google/benchmark) libary. It also provides a helper script for comparing the benchmark performance of two branches. An guide for setup and usage, as well as a list of benchmarks and executables can be found in this README.

## Manual setup and basic usage

1. If not already done, set up a Helios development installation as desribed in the [Helios Package README](https://github.com/3dgeo-heidelberg/helios/blob/alpha-dev/README.md#development-installation)
2. Compile helios using CMake instead of pip: 

    ```
    mkdir build
    cd build
    cmake -DBUILD_BENCHMARKS=ON -DCMAKE_CXX_FLAGS=-fno-omit-frame-pointer ..
    make
    cd ..
    ```

3. Execute the benchmark. Benchmark executables are found in ```build/benchmarks/```, and each executable can be run via ```./build/benchmarks/<benchmark_executable_name>```. This will by default generate an output in the console, showing average time per operation, average CPU time per operation and number of iterations for each benchmark in the executable.

**Note**: Most benchmark executables need to be executed from the Helios root directory to work correctly.

## Workflow for comparing branches

To analyze the performance changes from one branch to another, it is useful to compare benchmark outputs in a more systematic manner. Google Benchmark provides a comparison python script for this task.

For a more convenient usage of the comparison script, this directory provides two helper scripts: ```initial_setup.sh```, which has to be executed once to download the necessary files and create the directory structure, and ```comparison_workflow.sh```, which executes and compares benchmarks for two different branches, and stores the results.

### ```initial_setup.sh```

This script can be executed via ```./initial_setup.sh```, and it does the following:

- Creates the ```benchmarks/compare/``` directory
- Downloads the ```compare.py``` script, as well as its ```gbench``` dependencies from [Google Benchmark Tools GitHub](https://github.com/google/benchmark/tree/main/tools)

The comparison script requries ```scipy``` and ```numpy``` to run, which are also dependencies of helios, so make sure to have a development conda environment set up.

### ```comparison_workflow.sh```

#### Description

This script can be executed via ```./comparison_workflow.sh <branch_a> <branch_b>```, and it does the following:

- Deletes any previous .json files found in ```benchmarks/compare/```. Make sure to copy any comparison results that you want to keep to a different directory.
- For each branch <branch_a> and <branch_b>:
    - Git checkout to branch
    - Recompiles with cmake as described above, in a new build directory ```build_<branch_name>```. If a build directory with said name already exists, it is cleared.
    - Runs all benchmarks with ```<benchmark_exe_name> --benchmark_repetitions="$repetitions" --benchmark_out_format=json --benchmark_out="benchmarks/compare/<out_file_name>.json"```, which repeats the benchmark and stores the output in a json file
- For each produced benchmark output:
    - Runs the comparison script via ```python3 compare.py -d "<comparison_out_file_name>.json" benchmarks "<out_file_branch_a>" "<out_file_branch_b>"```, which also stores the comparison output in a json file

#### Options

The ```comparison_workflow.sh``` script provides the following options:\

- ```-f``` : Regex filter to be passed to the benchmark executables. For example, if only fullwaveform_digest_intersections_benchmark and base_energy_model_compute_power_benchmark should be compared, pass ```-f"fullwaveform_digest|base_energy"``` or a similar regex to this argument. Benchmark executables which do not contain a matching benchmark are skipped
- ```-j``` : Number of procs for the ```make``` command. Defaults to ```nprocs```
- ```-n``` : Number of repetitions to do for each benchmark. The higher the number of repetitions, the more reliable the result of the comparison will be. Defaults to 30

#### Output Explanation

The output from ```compare.py``` includes the following information:

- **For each benchmark**
    - **For each repetition of said benchmark**
        - *Time*: Average time per operation difference between branch b and branch a for this repetition
        - *CPU Time*: Average CPU time per operation difference between branch b and branch a for this repetition
        - *Time Old*: Average time per operation branch a
        - *Time New*: Average time per operation branch b
        - *CPU Old* Average CPU time per operation branch a
        - *CPU New* Average CPU time per operation branch b
    - *pvalue*: p-value from a U-test, with the null hypothesis that there is no significant difference between branch a and branch b. If this p-value is below the significance level (default 0.05), the null hypothesis is rejected, meaning that the benchmark performances are significantly different
    - *mean*: Relative difference in mean execution time (branch b - branch a)
    - *median*: Relative difference in median execution time
    - *stddev*: Relative difference in standard deviation of the execution time
    - *cv*: Coefficient of Variation, i.e. ratio of standard deviation to mean

More information on the output and on more execution options can be found in the [documentation of compare.py](https://google.github.io/benchmark/tools.html).


## List of benchmarks and executables

| Benchmark | Executable | Description |
| --------- | ---------- | ----------- |
| base_energy_model_compute_received_power_benchmark | base_energy_model_compute_received_power_b | Tests the performance of BaseEnergyModel::computeReceivedPower. Sets up a minimal SingleScanner/ScanningDevice context and repeatedly evaluates the received power for fixed, synthetic arguments. |
| fullwaveform_compute_subrays_benchmark | fullwaveform_compute_subrays_b | Tests the performance of the FullWaveformPulseRunnable::computeSubrays. It sets up a mock Scene, SingleScanner and FullWaveformPulseRunnable. The benchmark does not test the performance of testIntersection, and instead returns nullptr for all subray intersections. |
| fullwaveform_digest_intersections_benchmark | fullwaveform_digest_intersections_b | Tests the performance of FullWaveformPulseRunnable::digestIntersections. It builds a small mock scene + scanner setup and feeds a fixed set of synthetic reflections/intersections into digestIntersections. The goal is to measure the full-waveform digestion path, without including ray casting costs, which are tested via the computeSubrays benchmark. |
| fullwaveform_find_intersection_benchmark | fullwaveform_find_intersection_b | Tests the performance of the FullWaveformPulseRunnable::findIntersection method. Unlike the computeSubrays benchmark, this focuses on the ray-scene intersection path by repeatedly calling findIntersection against a small, in-memory triangle-grid scene that is finalized (KD-Grove built) once during setup. |
| fullwaveform_find_intersection_miss_benchmark | fullwaveform_find_intersection_b | Version of fullwaveform_find_intersection_benchmark that exercises the intersection traversal work when the ray intersects the scene AABB but does not intersect any primitive. It uses a sparse scene with a large AABB and a few tiny triangle patches placed in the corners, then shoots a ray through the empty center. |
| scanning_device_sim_step_benchmark | scanning_device_sim_step_b | Tests the performance of the doSimStep method of ScanningDevice. The benchmark sets up a ScanningDevice with example specifications, with an appropriate PolygonBeamDeflector and ScannerHead. The benchmark isolates the performance of the ScannerHead, BeamDeflector and ScanningDevice parts of the SimStep, and does not include pulse computation. |
| macro_playback_benchmark | macro_playback_b | **This is a macro benchmark, not intended for microbenchmarking performance analyses!** This benchmark is a macro benchmark for an entire SurveyPlayback. It follows the setup of a SurveyPlayback in LidarSim.cpp with the tls_toyblocks. |
