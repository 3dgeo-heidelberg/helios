# HELIOS ++ build instructions

## Table of contents
1. [Legacy version](#legacy-version)
1. [Install](#install)
    1. [Dependencies](#dependencies)
    2. [Install on Linux](#install-on-linux)
    3. [Install on Linux with PyHelios Support](#install-on-linux-with-pyhelios-support) 
    4. [Install on Linux with Visual Debug](#install-on-linux-with-visual-debug)
    5. [Install on Windows](#install-on-windows)
1. [Building the Wiki](#wiki)
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
For instance, to compile using 6 threads:
```
make -j 6
```

### Install on Linux with PyHelios support

In order to be able to compile the project enabling Python Bindings support,
all the libraries must be built from scratch:

### Helios Dependencies

- ```apt install cmake make gcc g++ xz-utils python3 python3-pip libpython3.8-dev unzip```

#### Armadillo

- ```apt install liblapack-dev```
- ```cd helios/lib```
- ```wget -O armadillo.tar.xz http://sourceforge.net/projects/arma/files/armadillo-10.6.2.tar.xz && tar xf armadillo.tar.xz```
- ```mv armadillo-10.6.2 armadillo && cd armadillo```
- ```./configure -DCMAKE_INSTALL_PREFIX=. && make && sudo make install```

#### GLM

Can be installed both by means of a package manager or building from source:

- ```apt install libglm-dev```

or

- ```cd helios/lib```
- ```wget https://github.com/g-truc/glm/releases/download/0.9.9.8/glm-0.9.9.8.zip && unzip glm-0.9.9.8 && rm glm-0.9.9.8.zip```
- ```cd glm && cmake . && make```

When building from source, it is mandatory to rename the GLM directory to lib/glm

#### LASTools

- ```cd helios/lib```
- ```wget https://lastools.github.io/download/LAStools.zip && unzip LAStools.zip```
- ```cd LAStools && cmake .  && make```

#### Boost

The linkage against Boost libraries can be performed both statically and dynamically.

- ```apt install zlib1g```
- ```wget https://boostorg.jfrog.io/artifactory/main/release/1.76.0/source/boost_1_76_0.tar.gz```
- ```tar -xzvf boost_1_76_0.tar.gz```
- ```mv boost_1_76_0 boost && cd boost```
- ```./bootstrap.sh --with-python=python3.8``` In order to use a custom Python installation, please see [Custom Python Installation](#CustomPython)
- ```./b2 cxxflags=-fPIC```

If you want static linkage, the boost installation ends here.

For allow dynamic linkage, execute the following command ||
IMPORTANT: Remove completely any previous existing Boost installation before continue.

- ```./b2 install```

#### GDAL
- ```apt install pkg-config libsqlite3-dev sqlite3 libtiff5-dev libcurl4-openssl-dev```
- ```wget http://download.osgeo.org/proj/proj-8.0.0.tar.gz https://github.com/OSGeo/gdal/releases/download/v3.3.0/gdal-3.3.0.tar.gz --no-check-certificate```
- ```tar -xzvf proj-8.0.0.tar.gz && tar -xzvf gdal-3.3.0.tar.gz```
- ```mv gdal-3.3.0 gdal && mv proj-8.0.0 proj```
- ```cd proj && ./configure --prefix=$PWD/../gdal/projlib && make && make install```
- ```cd ../gdal && ./configure --prefix=$PWD --with-proj=$PWD/projlib && make && make install```

If you are using a custom Python installation, we must provide GDAL its executable path using `--with-python=/path/to/python`:

- ```./configure --prefix=$PWD --with-proj=$PWD/projlib --with-python=/home/miniconda3/envs/py38/bin/python3.8 && make && make install```


#### PyHelios Dependencies

- ```apt install libgl1-mesa-dev```
- ```pip3 install open3d```

Note: The PyHelios scripts can be executed which whatever version of Python3 you want to use.

### Helios project configuration and compilation

Back to the helios root directory, configure the project:

- Linking Boost statically:

```cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=1 -DPYTHON_VERSION=38 .```

- Linking Boost dynamically:

```cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=1 -DPYTHON_VERSION=38 -DBOOST_DYNAMIC_LIBS=1```

- Compiling against a custom Python installation:

```cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=1 -DPYTHON_VERSION=36 -DPYTHON_PATH=/home/miniconda3/envs/py36/ .```

Finally, compile Helios++:

```make -jn``` where ```n``` is the amount of threads to be used in the compilation.


In order to execute PyHelios scripts, libhelios.so path must be added to PYTHONPATH (default location: helios root directory):

```export PYTHONPATH=$PYTHONPATH:/path/to/helios```

Finally, to execute the demo scene using PyHelios:

```python3 helios/pyhelios_demo/helios.py pyhelios_demo/custom_als_toyblocks.xml```


#### Custom Python Installation <a name="CustomPython"></a>

Both Helios and Boost depends on Python, and must be compiled accordingly.

##### Boost
- Create a user-config.jam in your $HOME directory.
- Add the following lines, adapting the paths to your Python paths and version:
```
using python
	: 3.8
	: /home/miniconda3/envs/py38/bin/python3.8 # Path to pythonX.Y executable
	: /home/miniconda3/envs/py38/include/python3.8 # Path to pyconfig.h location
	: /home/miniconda3/envs/py38/libs # Path to libpythonX.Y.so location
;
```
- Build Boost with the following commands:
  - ```cd lib/boost```
  - ``` ./bootstrap.sh```
  - ``` ./b2 --user-config=/path/to/user-config.jam cxxflags=-fPIC python=3.8```
   
For allowing dynamic Boost linkage, execute ```sudo ./b2 install```

##### Helios
- The path to the custom Python root directory must be provided to CMake:

```cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=1 -DPYTHON_VERSION=38 -DPYTHON_PATH=/home/miguelyermo/miniconda3/envs/py38 . ```
   
Note that -DPYTHON_VERSION must match the chosen Python installation. If no path is provided, CMake will attempt to use default Python installation.

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

> **Disclaimer: The manual is written asumming the compilation is done with Microsoft Visual Studio 20xx.** Use other compilers at your own risk. 

To obtain the source just clone it from the repository
```
git clone https://github.com/3dgeo-heidelberg/helios.git
```

or download the source code for the desired Helios++ version from [here](https://github.com/3dgeo-heidelberg/helios/tags).
You should know have a directory called `helios` containing the source files.

#### Solving dependencies

Helios++ depends on several libraries, which must be installed one by one.

---
##### Boost C++

###### Configuring the project
First we need to download a version of Boost C++ library greater than or equal to 1.71:

[Download Boost](https://www.boost.org/users/history/)

Unzip it inside the `lib` folder and rename it to `boost`.

We also need to download zlib and unzip it inside the `lib/boost` folder.

[Download zlib](https://www.zlib.net/zlib1211.zip)

Open a Command Prompt terminal in `lib/boost` and execute the following command:

```bootstrap.bat```


###### Compiling Boost
If you are **not planning** to compile with Python Binding support, execute:

```
b2.exe -j6 -sNO_ZLIB=0 -sZLIB_INCLUDE="zlib-1.2.11" -sZLIB_SOURCE="zlib-1.2.11" address-model=64 link=static
```
**REMEMBER** to change the zlib version in above command if you downloaded a different one


If you are **planning** to compile with Python Bindings support, Boost installation is not over.

---
##### Boost C++ with python bindings
Boost needs to be compiled with support for Python to be able to use python bindings.
The version of boost-python needs to be adapted to the targeted python version.
To configure different versions, edit or create a file called `user-config.jam`

Add the following lines to the file (adapting the paths to your python installations, the example is using [miniconda](https://docs.conda.io/en/latest/miniconda.html).
For demonstration purposes, several Python versions are included. You can pick just one and continue.
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

Save the file in the *user home directory*, e.g. `C:\users\yourname\`.
If you want to use other location, add `--user-config=/path/to/user-config.jam` with your `user-config.jam` location to the `b2` command below.
Then build **Boost C++** using the following command:

```
b2.exe -j6 -sNO_ZLIB=0 -sZLIB_INCLUDE="zlib-1.2.11" -sZLIB_SOURCE="zlib-1.2.11" address-model=64 link=static python=3.6,3.7,3.8,3.9
```

- Remember to change the `zlib` version if you downloaded a different one.
- Remember to adapt the Python version to the one you included in `user-config.jam`

---
##### OpenGLM Mathematics
Simply download the [OpenGLM mathematics library](https://github.com/g-truc/glm/tags)
and unzip it inside `helios/lib`. 

Rename `lib/glm-x.x.x.x/` to `lib/glm/`

---
##### GDAL with PROJ

First, we need to download GDAL compiled binaries and headers, and the 
generic installer for the GDAL core components from [here](https://www.gisinternals.com/release.php)

- Choose the appropriate option matching your compiler and architecture (MSVC2019, x64 for instance) for
GDAL 3.x.x version. For example, `release-1928-x64-gdal-3-x-x-mapserver-7-6-4`
- Download the files `release-1928-x64-gdal-3-x-x-mapserver-7-6-4-libs.zip` and
`gdal-303-1928-x64-core.msi`.
- Create a folder named gdal inside the lib folder (lib/gdal) and extract the contents of the zip file there. `lib/gdal/include` and `lib/gdal/lib` must exist after the extraction.
- Install the .msi file (By default, `C:\Program Files\GDAL`). **Add the installation directory to your PATH**
- Finally, rename the `lib/gdal/include/boost` folder to `lib/gdal/include/boost-gdal` to avoid conflicts when building helios++

---
##### LAStools library

###### Configuring the project <a name="CMake"></a>
Download [LAStools](https://rapidlasso.com/lastools/) and unzip it inside lib folder.

Using [CMakeGUI](https://cmake.org/download/) we can configure and generate the LAStools project. If you prefer the command line version,
just execute `cmake .` inside `lib/LAStools` to generate the project. 

###### Compiling LASTools
Once the project is generated we can compile it using Visual Studio. 
Open `LASTools.sln`, change the project mode to *Release* and build. 
If the build was successful, `lib/LAStools/LASlib/lib/Release/LASlib.lib` must exist.

In case you have used a compiler different than MSVC, you will need to manually place
the `LASlib.lib` file at `lib/LAStools/LASlib/lib/LASlib.lib` or at `lib/LAStools/LASlib/lib/Release/LASlib.lib`.
Both locations are supported.

---
##### Armadillo

It is recommended to use Armadillo with high performance versions of BLAS and LAPACK.
In consequence, here is exposed how to compile Helios++ with Armadillo using Scilab x64,
which provides high performance implementations of BLAS and LAPACK.
Of course, it is possible to use different implementations.

###### Configuring the project

- Scilab: First, download and install [Scilab](https://www.scilab.org).
In this case we are going to use [Scilab 6.1.0 x64](https://www.scilab.org/download/6.1.0/scilab-6.1.0_x64.exe).
By default, it is installed at `C:\Program Files\scilab-6.1.0`. **Add the installation directory to your PATH**
- Armadillo: 
Now it is necessary to download [Armadillo](http://arma.sourceforge.net/download.html)
and decompress the source files inside *lib/armadillo*.
In this case we are going to use [Armadillo 10.6.2](http://sourceforge.net/projects/arma/files/armadillo-10.6.2.tar.xz).

###### Compiling Armadillo

Once Armadillo has been decompressed it is time to use CMake to configurate and
generate the project like you did [here](#CMake). Then, we need to open `armadillo.sln` with Microsoft
Visual Studio and compile it in RELEASE mode for x64 architecture.
If the build was successful, `lib/armadillo/Release/armadillo.lib` must exist.

---
##### Helios++

###### Configuring the project

At the `helios` directory, generate the project as you did [here](#CMake).

You can use certain flags together with CMake/CMakeGUI to configure the python bindings compilation:

```PYTHON_BINDING``` Set it to TRUE or 1 to compile with python bindings support. Set it to FALSE or 0 to ignore python bindings.

```PYTHON_VERSION``` Set it to the python version you would like to use. For instance 38. If it is not specified,
python version 37 is considered by default.

```PYTHON_PATH``` Use it if you need to specify the path to a python installation that is not the system default.

```LAPACK_LIB``` Set here the path to custom LAPACK library.
When specifying LAPACK_LIB it is not necessary to link it at Visual Studio IDE, since it will be already included.


For example, to compile Helios with Python Bindings support, using `python3.6` located in `D:\Miniconda3\envs\py36\python.exe`:

```
cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=1 -DPYTHON_VERSION=36 -DPYTHON_PATH=D:\Miniconda3\envs\py36 .
```

Finally, we only need to link at helios project. Open `helios.sln`, right click on *helios* at solution explorer and then properties:
1) At `Linker -> General -> Additional Library Directories` add the path to the folder where
scilab "dll" files are contained.
For instance: `C:\Program Files\scilab-6.1.0\bin`.
2) At `Linker -> Input -> Additional Dependencies` it is necessary to add `lapack.lib` and `blasplus.lib` 
For instance: `C:\Program Files\scilab-6.1.0\bin\lapack.lib` and `C:\Program Files\scilab-6.1.0\bin\blasplus.lib`

Repeat this process for *pyhelios* at solution explorer.

###### Compiling Helios

Once the project is generated we can compile it using Visual Studio. Open `helios.sln`, change the project mode to *Release* and build.
If the compilation was sucessful, the following files must exist in `helios/Release` directory:
```
 - helios.exe 
 - helios.exp 
 - helios.lib
 - pyhelios.exp 
 - pyhelios.lib
 ```


### How to execute
To execute Helios++ (helios.exe binary) it is necessary to have both GDAL dll and LAPACK dll in the path. If you followed the instructions, they must be already included in the PATH.
If you are using scilab you are going to need linking all scilab dll files.
By default they are located inside the same directory. But if you have moved them to your own folders, remember to include them all.

**By default, C:\Program Files\GDAL and C:\Program Files\scilab-6.1.0\bin must be added to the PATH.**

**IMPORTANT: GDAL and SCiLab share the libcurl.dll dependency. It is mandatory GDAL uses its own libcurl.dll implementation. To achieve this, just be sure that the GDAL path is before the SCILAB path in the PATH. If you are planning to move the shared libraries to another location, be sure that the libcurl.dll version you use is the provided by the GDAL installation.**

There are two ways to modify the PATH:

#### Session PATH

You can modify the path for the current session, for instance for the active command line.
This way, changes are not going to be permanent neither for the user nor for the system.
Always that you open a new session, you are going to need to repeat this process.
It can be done with following command:
```
set PATH=%PATH%;<path to gdal dll folder>;<path to lapack dll folder>
```

#### User/System PATH

It is possible to also modify the path at advanced system properties.
For this right click on This-PC, then go to Properties and find advanced system properties.
There you can open the environment variables dialogue. Here you have two options.
You can modify user variables, so changes will be applied only for current user.
Or you can modify system variables, so changes will be applied for all users.
Once you have decided where to make changes, just find the PATH variable and append
at the end:
```
;<path to gdal dll folder>;<path to lapack dll folder>
```

---
## Wiki
You can build the HELIOS++ Wiki locally as html files using `mkdocs`. For this, the wiki is included as a `git submodule`.

Make sure you have the latest copy of the submodule checked out with `git submodule update --init --recursive`

Then change to the respective directory: `cd wiki-repo`

And run mkdocs (you may need to install it and any required packages using `pip` or other python package managers):

`python -m mkdocs build`

The local wiki build is then generated in the `site` subdirectory.

---
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
