FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /home
RUN apt update && apt install -y gcc g++ cmake make git wget unzip && \
git clone https://github.com/3dgeo-heidelberg/helios.git

WORKDIR /home/helios/lib

# Download and extract needed libs
RUN wget http://lastools.github.io/download/LAStools.zip \
http://download.osgeo.org/proj/proj-8.0.0.tar.gz \
https://github.com/OSGeo/gdal/releases/download/v3.3.0/gdal-3.3.0.tar.gz --no-check-certificate \
https://boostorg.jfrog.io/artifactory/main/release/1.76.0/source/boost_1_76_0.tar.gz && \
unzip LAStools.zip && rm LAStools.zip && tar -xzvf proj-8.0.0.tar.gz && tar -xzvf gdal-3.3.0.tar.gz && tar -xzvf boost_1_76_0.tar.gz

# Install LASTools
WORKDIR /home/helios/lib/LAStools
RUN cmake . && make

# Install Proj
WORKDIR /home/helios/lib/proj-8.0.0
RUN apt install -y pkg-config libsqlite3-dev sqlite3 libtiff5-dev libcurl4-openssl-dev && \
./configure && make -j 6 && make install

# Install GDAL
WORKDIR /home/helios/lib/gdal-3.3.0
RUN ./configure && make -j 6 && make install

# Install Boost
WORKDIR /home/helios/lib/boost_1_76_0
RUN apt install -y libpython3.8-dev python3 python3-pip && \
./bootstrap.sh --with-python=python3.8 && ./b2 cxxflags=-fPIC && ./b2 install

# Install dependencies
WORKDIR /home/helios
RUN apt -y install libarmadillo-dev libglm-dev libglu1-mesa-dev

# Set PYTHONPATH
ENV PYTHONPATH=/home/helios

# Compile Helios with PyBindings active
RUN cmake -DPYTHON_BINDING=1 -DPYTHON_VERSION=38 . && make -j 6

# Install PyHelios dependencies
RUN python3 -m pip install open3d

# Clean
WORKDIR /home/helios/lib
RUN rm *.tar.gz

WORKDIR /home/helios
