# HELIOS ++ build instructions

## Table of contents
1. [Legacy version](#legacy-version)
1. [Install](#install)
    1. [Dependencies](#dependencies)
    1. [Install on Linux](#install-on-linux)
    1. [Install on Windows](#install-on-windows)
1. [Usage](#usage)
1. [License](#license)

## Legacy version
HELIOS, the precedessor of HELIOS++, can still be found at https://github.com/GIScience/helios

## Install

### Dependencies
[CMake is necessary to build the code](https://cmake.org/) (Version 3.15 or greater is recommended)

[Boost C++ Libraries version 1.71 or greater](https://www.boost.org/)

[GDAL with PROJ version 3 or greater](https://gdal.org/)

[OpenGL Mathematics library version 0.9.9 or greater](https://glm.g-truc.net)

[TinyXML2 library](https://github.com/leethomason/tinyxml2) (Already distributed with source code)

### Install on Linux

To obtain the source just clone it from the repository
```
git clone <URL to the public repository>
```

#### Solving dependencies

```
sudo apt install cmake libboost-dev libboost-system-dev libboost-thread-dev libboost-regex-dev libboost-filesystem-dev libboost-iostreams-dev libgdal-dev libglm-dev libarmadillo-dev
```

##### LAStools library

We need to download [LAStools](https://rapidlasso.com/lastools/) and unzip it inside lib folder.

Now we need to get into the LAStools directory and use CMake and Makefile to build
```
cmake . && make
```
If successfully built the file lib/LAStools/LASlib/lib/libLASlib.a must exist now.

#### Compiling source
Change directory to helios-plusplus root folder
```
cd helios-plusplus
```
Use cmake to configure and generate build files
```
cmake .
```
If you want to compile enabling python bindings, use this cmake command instead
```
cmake -DPYTHON_BINDING=1 .
```
Also, if you need to specify a concrete python version or even provide the path to a concrete python installation,
you can use following flags with cmake (you can change version or path to match yours)
```
cmake -DPYTHON_BINDING=1 -DPYTHON_VERSION=38 -DPYTHON_PATH=/home/user/mypython .
```
To compile providing a concrete implementation of LAPACK library, use this cmake command
```
cmake -DLAPACK_LIB=/home/user/mylapack.so
```
Use make to build
```
make
```
Build can be accelerated using multithreading.
The number of threads can be specified with -j argument.
For instance, to compil using 6 threads:
```
make -j 6
```

### Install on Windows

To obtain the source just clone it from the repository
```
git clone <URL to the public repository>
```

#### Solving dependencies

##### Boost C++
First we need to download a version of Boost C++ library greater than or equal to 1.71 from https://www.boost.org/users/download/

Then we need to unzip it inside the lib folder and rename it to boost.
For windows we also need to download [zlib](https://www.zlib.net) and unzip it inside the boost folder.
It can be obtained from https://www.zlib.net/zlib1211.zip


Now we must get into the lib/boost directory and execute following commands
```
bootstrap.bat
b2.exe -j6 -sNO_ZLIB=0 -sZLIB_INCLUDE="zlib-1.2.11" -sZLIB_SOURCE="zlib-1.2.11" address-model=64 link=static
```
**REMEMBER** to change the zlib version in above command if you download a different one

##### OpenGLM Mathematics
Simply download the [OpenGLM mathematics library](https://github.com/g-truc/glm/tags)
and unzip it inside the helios-plusplus lib folder


##### GDAL with PROJ

First create a folder named gdal inside the lib folder and download a precompiled version of the library,
for instance from http://download.gisinternals.com/sdk/downloads/release-1900-x64-gdal-3-0-4-mapserver-7-4-3-libs.zip

Place header files (.h and .hpp) in the directory lib/gdal/include/ and lib files inside lib/gdal/lib/

Finally, rename the lib/gdal/include/boost folder to lib/gdal/include/boost-gdal to avoid
conflicts when building helios++

##### LAStools library

We need to download [LAStools](https://rapidlasso.com/lastools/) and unzip it inside lib folder.

Using [CMakeGUI](https://cmake.org/download/) we can configure and generate the LAStools project.

Once the project is generated we can compile it using Visual Studio. Just remember to change the
project mode to Release. If it built successfully we should have a lib file at lib/LAStools/LASlib/lib/Release/
called LASlib.lib. In case you have used a compiler different than MSVC, you will need to manually place
the LASlib.lib file at lib/LAStools/LASlib/lib/LASlib.lib or at lib/LAStools/LASlib/lib/Release/LASlib.lib.
Both locations are supported.

##### Armadillo

It is recommended to use Armadillo with high performance versions of BLAS and LAPACK.
In consequence, here is exposed how to compile helios++ with Armadillo using Scilab x64,
which provides high performance implementations of BLAS and LAPACK.
Of course, it is possible to use different implementations.

First, we need to download and install [Scilab](https://www.scilab.org).
In this case we are going to use [Scilab 6.1.0 x64](https://www.scilab.org/download/6.1.0/scilab-6.1.0_x64.exe).

Now it is necessary to download [Armadillo](http://arma.sourceforge.net/download.html)
and decompress it inside *lib/armadillo*.

Using [Armadillo 9.900.1](http://sourceforge.net/projects/arma/files/armadillo-9.900.1.tar.xz)
or newer is recommended.

Once Armadillo has been decompressed it is time to use CMake to configurate and
generate the project. Then, we need to open *armadillo.sln* with Microsoft
Visual Studio and compile it in RELEASE mode for x64 architecture.

Finally, we only need to link at helios project. In case of Microsoft Visual
Studio this can be done in 2 easy steps at helios properties (accessible through
right click on *helios* at solution explorer and then properties):
1) At `Linker -> General` it is recommended to add path to the folder where
scilab dll files are contained.
For instance: *C:\Program Files\scilab-6.1.0\bin*
2) At `Linker -> Input` it is necessary to add path to the folder where scilab
lib files are contained.
For instance: *C:\Program Files\scilab-6.1.0\bin*

#### Compiling source

To compile from windows is recommended to use [CMakeGUI](https://cmake.org/download/)

From CMakeGUI specify the path to the helios-plusplus folder and then configure and generate the project.

You can use certain flags together with CMakeGUI to configure the python bindings compilation:

```PYTHON_BINDING``` Set it to TRUE or 1 to compile with python bindings support. Set it to FALSE or 0 to ignore python bindings.

```PYTHON_VERSION``` Set it to the python version you would like to use. For instance 38. If it is not specified,
python version 37 is considered by default.

```PYTHON_PATH``` Use it if you need to specify the path to a python installation that is not the system default.

It is also possible to specify the path to desired LAPACK library, so Armadillo will be linked with it:

```LAPACK_LIB``` Set here the path to custom LAPACK library.
When specifying LAPACK_LIB it is not necessary to link it at Visual Studio IDE, since it will be already included.


Once the project has been generated it can be compiled using visual studio code.

**NOTICE** when building form visual studio code it is necessary to specify the release configuration,
otherwise compilation will not work.

#### How to execute
To execute helios++ (helios.exe binary) it is necessary to have both GDAL dll and LAPACK dll in the path.
If you are using scilab you are going to need linking all scilab dll files.
By default they are located inside the same directory. But if you have moved them to your own folders,
remember to include them all.

There are two ways to modify the path:

**Session path**

You can modify the path for the current session, for instance for the active command line.
This way, changes are not going to be permanent neither for the user nor for the system.
Always that you open a new session, you are going to need to repeat this process.
It can be done with following command:
```
set PATH=%PATH%;<path to gdal dll folder>;<path to lapack dll folder>
```

**User/System path**

It is possible to also modify the path at advanced system properties.
For this right click on This-PC, then go to Properties and find advanced system properties.
There you can open the environment variables dialogue. Here you have two options.
You can modify user variables, so changes will be applied only for current user.
Or you can modify system variables, so changes will be applied for all users.
Once you have decided where to make changes, just find the PATH variable and append
at the end:
```
;<path to gdall dll folder>;<path to lapack dll folder>
```

## Usage
Helios++ can be invoked with following syntax:

```
helios --help
    Show the help for helios++ usage

helios --test
    Perform necessary tests to check everything works as expected

helios <survey_file_path> [OPTIONAL ARGUMENTS]
    Perform requested simulation.

    NOTICE specifying the path to the survey specification file is mandatory

    Available general OPTIONAL ARGUMENTS are:
        --assets <directory_path>
            Specify the path to assets directory
        --output <directory_path>
            Specify the path to output directory
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
        --lasOutput
            Specify the output point cloud must be generated using LAS format
        --zipOutput
            Specify the output point cloud and fullwave must be zipped
        --lasScale
            Specify the scale factor used to generate LAS output
        -j OR --njobs OR --nthreads <integer>
            Specify the number of simultaneous threads to be used to compute
            the simulation
            If it is not specified or it is specified as 0, then all available
            threads will be used to compute the simulation
        --rebuildScene
            Force scene rebuild even when a previosly built scene is available
        --disablePlatformNoise
            Disable platform noise, no matter what is specified on XML files
        --disableLegNoise
            Disable leg noise, no matter what is specified on XML files

    Available logging verbosity OPTIONAL ARGUMENTS are:
        --silent
            Nothing will be reported
        -q OR --quiet
            Only errors will be reported
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
./helios data/surveys/demo/tls_arbaro_demo.xml
```

**WINDOWS**
```
helios.exe data/surveys/demo/tls_arbaro_demo.xml
```

## License

See [LICENSE.md](LICENSE.md)
