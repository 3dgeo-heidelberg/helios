FROM ubuntu:20.04

# Install base utilities
RUN apt-get update && \
    apt-get install -y build-essential && \
    apt-get install -y wget && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Helios installation
ENV DEBIAN_FRONTEND=noninteractive

# Update packages
WORKDIR /home
RUN apt update && apt install -y gcc g++ cmake make git wget unzip xz-utils


# Clone repository
ARG branch
RUN git clone --branch ${branch} https://github.com/3dgeo-heidelberg/helios.git && cd helios && git status

WORKDIR /home/helios/lib

# Download and extract needed libs
RUN wget http://lastools.github.io/download/LAStools.zip \
http://download.osgeo.org/proj/proj-8.0.0.tar.gz \
https://github.com/OSGeo/gdal/releases/download/v3.3.0/gdal-3.3.0.tar.gz --no-check-certificate \
https://boostorg.jfrog.io/artifactory/main/release/1.76.0/source/boost_1_76_0.tar.gz \
https://github.com/g-truc/glm/releases/download/0.9.9.8/glm-0.9.9.8.zip \
http://sourceforge.net/projects/arma/files/armadillo-10.6.2.tar.xz && \
unzip LAStools.zip && rm LAStools.zip && tar -xzvf proj-8.0.0.tar.gz && \
tar -xzvf gdal-3.3.0.tar.gz && tar -xzvf boost_1_76_0.tar.gz && \
unzip glm-0.9.9.8.zip && rm glm-0.9.9.8.zip && \
tar xf armadillo-10.6.2.tar.xz && mv armadillo-10.6.2 armadillo && rm armadillo-10.6.2.tar.xz

# Install LASTools
WORKDIR /home/helios/lib/LAStools
RUN cmake . && make

# Install GLM
WORKDIR /home/helios/lib
RUN cd glm && cmake . && make -j 6

# Install Armadillo
WORKDIR /home/helios/lib/armadillo
RUN apt install -y liblapack-dev libblas-dev && \
./configure -DCMAKE_INSTALL_PREFIX=. && make && make install

# Install Proj
WORKDIR /home/helios/lib
RUN apt install -y pkg-config libsqlite3-dev sqlite3 libtiff5-dev libcurl4-openssl-dev && \
mv proj-8.0.0 proj && cd proj && \
./configure --prefix=/home/helios/lib/gdal/projlib && make -j 15 && make install

# Install GDAL
WORKDIR /home/helios/lib
RUN mv gdal-3.3.0/* gdal/ && cd gdal && ./configure --prefix=/home/helios/lib/gdal --with-proj=/home/helios/lib/gdal/projlib && make -j 15 && make install

# Make libgdal.so and libproj.so link against libcurl4 installed by libcurl4-openssl-dev
RUN apt install patchelf && patchelf --replace-needed libcurl.so.4 /usr/lib/x86_64-linux-gnu/libcurl.so ./gdal/lib/libgdal.so && \
patchelf --replace-needed libcurl.so.4 /usr/lib/x86_64-linux-gnu/libcurl.so ./gdal/projlib/lib/libproj.so

# Manage Python Version
ARG pyv
ENV PYTHON_VERSION=$pyv
ENV ENV_NAME=pyhelios_env

# Install miniconda
ENV CONDA_DIR /opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
     /bin/bash ~/miniconda.sh -b -p /opt/conda

# Put conda in path so we can use it
ENV PATH=$CONDA_DIR/bin:$PATH

# Create the environment
    # Get enviroment file
COPY conda-environment.yml .
    # Get environment name from the conda-environment.yml file
RUN ENV_NAME=$(grep name: conda-environment.yml | cut -d\  -f2)
    # Its easier to change the environment name that retrieve it and save it in a variable.
RUN sed -i "s/name:.*/name: ${ENV_NAME}/" conda-environment.yml
    # Change Python version in the environment
RUN sed -i "s/python=.*/python=$PYTHON_VERSION/" conda-environment.yml

RUN conda update -n base conda && conda install -n base conda-libmamba-solver && conda config --set solver libmamba

RUN cat conda-environment.yml && conda config --set channel_priority strict
    # Create the conda environment
RUN conda env create -f conda-environment.yml

# Define environment path
ENV CONDA_ENV_DIR ${CONDA_DIR}/envs/${ENV_NAME}

# Install Boost
WORKDIR /home/helios/lib
RUN mv boost_1_76_0 boost
RUN echo "using python\n\t: ${PYTHON_VERSION}\n\t: ${CONDA_ENV_DIR}/bin/python\n\t: $(find "${CONDA_ENV_DIR}" -name "Python.h" -printf '%h\n')\n\t: ${CONDA_ENV_DIR}/lib\n;\n" >> ./boost/user-config.jam
RUN cat ./boost/user-config.jam
RUN cd boost && ./bootstrap.sh && ./b2 -j 15 --user-config=user-config.jam cxxflags=-fPIC python=${PYTHON_VERSION}

# Set PYTHONPATH
ENV PYTHONPATH=/home/helios

# Install PYHelios dependencies
RUN apt install -y libglu1-mesa-dev

# Compile Helios with PyBindings active
WORKDIR /home/helios
RUN cmake -DPYTHON_BINDING=1 -DPYTHON_VERSION=${PYTHON_VERSION} -DPYTHON_PATH=${CONDA_ENV_DIR} . && make -j 15

# Install PyHelios dependencies

# Clean
WORKDIR /home/helios/lib
RUN rm *.tar.gz

# Make sure we use the new created Python environment
ENV PATH /opt/conda/envs/${ENV_NAME}/bin:$PATH
WORKDIR /home/helios
