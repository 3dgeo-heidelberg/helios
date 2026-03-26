# Benchmarking setup

This directory provides several benchmarks for performance analyses using the [Google Benchmark](https://github.com/google/benchmark) libary. Benchmarks related to the same part of the Helios simulation are grouped together in one benchmark executable. A list of benchmarks and executables can be found at the bottom of this README.

## Setup and basic usage

1. If not already done, set up a Helios development installation as desribed in the [Helios Package README](https://github.com/3dgeo-heidelberg/helios/blob/alpha-dev/README.md#development-installation)
2. Compile helios using CMake instead of pip: 

    ```mkdir build
    cd build
    cmake -DBUILD_BENCHMARKS=ON ..
    make
    cd ..
    ```

3. Execute the benchmark. Benchmark executables are found in ```build/benchmarks/```, and each executable can be run via ```./build/benchmarks/<benchmark_executable_name>```. This will by default generate an output in the console, showing average time per operation, average CPU time per operation and number of iterations for each benchmark in the executable.

**Note**: Most benchmark executables need to be executed from the Helios root directory to work correctly.

## Workflow for comparing branches

To analyze the performance changes from one branch to another, it is useful to compare benchmark outputs in a more systematic manner. Google Benchmark provides a comparison python script for this task. If benchmark was installed via conda-forge, the comparison script as well as the required gbench package are not installed with it. The easiest setup is to clone the google benchmark repository to another location and set up an environment for running comparisons:

```git clone https://github.com/google/benchmark.git
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
9. ```python compare.py benchmarks <outfile_name_branch_a> <outfile_name_branch_b>```

This will generate an output for each benchmark, and for each repetition, that shows the time difference, CPU time difference, Old Time in branch a, New Time in branch b, Old CPU Time in branch a, and New CPU Time in branch b.

At the bottom of each benchmark, ```compare.py``` output shows the p-value from a Mann-Whitney


## List of benchmarks and executables

### Microbenchmarks
