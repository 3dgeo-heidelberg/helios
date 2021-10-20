# HELIOS ++ build instructions

## Table of contents
1. [Legacy version](#legacy-version)
1. [Install](#install)
    1. [Dependencies](#dependencies)
    2. [Install on Linux](#install-on-linux)
    3. [Install on Linux with PyHelios Support](#install-on-linux-with-pyhelios-support) 
    4. [Install on Linux with Visual Debug](#install-on-linux-with-visual-debug)
    5. [Install on Windows](#install-on-windows)
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

### Install on Linux with PyHelios support

In order to be able to compile the project enabling Python Bindings support,
all the libraries must be built from scratch:

### Helios Dependencies

- ```apt install cmake make gcc g++ libarmadillo-dev libglm-dev python3 python3-pip libpython3.8-dev unzip```


####GLM

Can be installed both by means of a package manager or building from source:

- ```apt install libglm-dev```

or

- ```wget https://github.com/g-truc/glm/releases/download/0.9.9.8/glm-0.9.9.8.zip && unzip glm-0.9.9.8 && rm glm-0.9.9.8.zip```
- ```cd glm && cmake . && make```

When building from source, it is mandatory to rename the GLM directory to lib/glm

#### LASTools

- ```cd helios/lib```
- ```wget https://lastools.github.io/download/LAStools.zip && unzip LAStools.zip```
- ```cd LAStools && cmake .  && make```

#### Boost

The linkage against Boost libraries can be performed both statically and dynamically.

- ```wget https://boostorg.jfrog.io/artifactory/main/release/1.76.0/source/boost_1_76_0.tar.gz```
- ```tar -xzvf boost_1_76_0.tar.gz```
- ```mv boost_1_76_0 boost && cd boost```
- ```./bootstrap.sh --with-python=python3.8```
- ```./b2 cxxflags=-fPIC```

If you want static linkage, the boost installation ends here.

For allow dynamic linkage, execute the following command ||
IMPORTANT: Remove completely any previous existing Boost installation before continue.

- ```./b2 install```

#### Proj
- ```apt install pkg-config libsqlite3-dev sqlite3 libtiff5-dev libcurl4-openssl-dev```
- ```wget http://download.osgeo.org/proj/proj-8.0.0.tar.gz```
- ```tar -xzvf proj-8.0.0.tar.gz```
- ```cd proj-8.0.0 && ./configure && make && make install```

#### GDAL
- ```wget https://github.com/OSGeo/gdal/releases/download/v3.2.1/gdal-3.3.0.tar.gz --no-check-certificate ```
- ```tar -xzvf gdal-3.3.0.tar.gz```
- ```cd gdal-3.3.0 && ./configure && make && make install```

#### PyHelios Dependencies

- ```pip3 install open3d```

Back to the helios root directory, compile:

- Linking Boost statically:

```cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=1 -DPYTHON_VERSION=38 .```

- Linking Boost dynamically:

```cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=1 -DPYTHON_VERSION=38 -DBOOST_DYNAMIC_LIBS=1```

In order to execute PyHelios scripts, libhelios.so path must be added to PYTHONPATH (default location: helios root directory):

```export PYTHONPATH=$PYTHONPATH:/path/to/helios```

Finally, to execute the demo scene using PyHelios:

```python3 helios/pyhelios_demo/helios.py pyhelios_demo/custom_als_toyblocks.xml```

### Install on Linux with Visual Debug
Linux users might compile Helios++ with a visual debug module used mainly for
development and debugging purposes. 

First step is building Helios libraries through development scripts in
*helios/scripts/build/* using script:
```
./build_all_libs.sh -c -v -w 6
```
The `-c` flag is used to include PCL (Point Cloud Library), the `-v` flag
will build VTK and use it later to build PCL and the `-w 6` argument requests
compilation acceleration using 6 threads. Of course, `-w n` is supported so
feel free to use preferred number of threads. There is one more flag that
can be used to specify the python version for Boost and PyHelios `-p 3.7`,
just in case.

Finally, when compiling Helios++ remember to call CMake with flag
`-DPCL_BINDING=1` so PCL and VTK are considered when building Helios. Notice
using PCL requires the C++ standard upgrades from C++11 to C++14, which differs
from base Helios building which only requires C++11.

### Install on Windows

To obtain the source just clone it from the repository
```
git clone https://github.com/3dgeo-heidelberg/helios.git
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

##### Boost C++ with python bindings
Boost needs to be compiled with support for python if the `HELIOS++` `pyhelios` bindings
should be compiled. The version of boost-python needs to be adapted to the targeted python version.
To configure different versions, edit or create a file called `user-config.jam` in the *user home directory*, e.g. `C:\users\yourname\`.

Add the following lines to the file (adapting the paths to your python installations, the example is using [miniconda](https://docs.conda.io/en/latest/miniconda.html)):
```
using python 
   : 3.6
   : D:\\Miniconda3\\envs\\py36\\python.exe
   : D:\\Miniconda3\\envs\\py36\\include #directory that contains pyconfig.h
   : D:\\Miniconda3\\envs\\py36\\libs    #directory that contains python36.lib
   ;
using python 
   : 3.7
   : D:\\Miniconda3\\envs\\py37\\python.exe
   : D:\\Miniconda3\\envs\\py37\\include #directory that contains pyconfig.h
   : D:\\Miniconda3\\envs\\py37\\libs    #directory that contains python37.lib
   ;
using python 
   : 3.8
   : D:\\Miniconda3\\envs\\py38\\python.exe
   : D:\\Miniconda3\\envs\\py38\\include #directory that contains pyconfig.h
   : D:\\Miniconda3\\envs\\py38\\libs    #directory that contains python38.lib
   ;
using python 
   : 3.9
   : D:\\Miniconda3\\envs\\py39\\python.exe
   : D:\\Miniconda3\\envs\\py39\\include #directory that contains pyconfig.h
   : D:\\Miniconda3\\envs\\py39\\libs    #directory that contains python39.lib
   ;
```

Then build **Boost C++** using the following command:

```
bootstrap.bat
b2.exe -j6 -sNO_ZLIB=0 -sZLIB_INCLUDE="zlib-1.2.11" -sZLIB_SOURCE="zlib-1.2.11" address-model=64 link=static -python=3.6,3.7,3.8,3.9
```

You can later decide which version to build `pyhelios` for, see [Compiling source](#compiling-source).

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

Using [Armadillo 10.6.2](http://sourceforge.net/projects/arma/files/armadillo-10.6.2.tar.xz)
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

<p>
<b><span style="color:red;">/!\</span>
WARNING
<span style="color:red">/!\</span></b>
Visual studio linking errors might occur. If it is your case, try explicitly
binding both LAPACK and BLAS at the IDE itself:
</p>

```
scilab-6.1.0/bin/lapack.lib
scilab-6.1.0/bin/blasplus.lib
```

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
