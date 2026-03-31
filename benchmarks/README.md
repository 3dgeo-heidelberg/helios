# Benchmarking setup

This directory provides several benchmarks for performance analyses using the [Google Benchmark](https://github.com/google/benchmark) libary. Benchmarks related to the same part of the Helios simulation are grouped together in one benchmark executable. A list of benchmarks and executables can be found at the bottom of this README.

## Setup and basic usage

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

To analyze the performance changes from one branch to another, it is useful to compare benchmark outputs in a more systematic manner. Google Benchmark provides a comparison python script for this task. If benchmark was installed via conda-forge, the comparison script as well as the required gbench package are not installed with it. The easiest setup is to clone the google benchmark repository to another location and set up an environment for running comparisons:

```
git clone https://github.com/google/benchmark.git
cd benchmark
conda create --name benchmark-compare --file requirements.txt
```

We will also make use of the following options for running benchmark executables:
- ```--benchmark_out_format=json```: Sets the output mode to json
- ```--benchmark_out=<outfile_name>```: Specifies the json output file name
- ```--benchmark_repetitions=<N>```: Sets the amount of repetitions to be done for each benchmark.

Comparison workflow of benchmark executable <benchmark_executable_name> from <branch_a> compared to <branch_b>:

1. ```git checkout <branch_a>```
2. Recompile helios on branch a using CMake with the respective flags as above
3. ```./build/benchmarks/<benchmark_executable_name> --benchmark_repetitions=27 --benchmark_out=<outfile_name_branch_a> --benchmark_out_format=json```
4. ```git checkout <branch_b>```
5. Recompile helios on branch b using CMake with the respective flags as above
6. ```./build/benchmarks/<benchmark_executable_name> --benchmark_repetitions=27 --benchmark_out=<outfile_name_branch_b> --benchmark_out_format=json```
7. ```cp <outfile_name_branch_a> <outfile_name_branch_b> <path_to_benchmark_installation/tools/>```. **Note that the file is copied to the ```tools``` folder**
8. ```conda activate benchmark-compare```
9. ```python compare.py -d compare_out.json benchmarks <outfile_name_branch_a> <outfile_name_branch_b>```

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
