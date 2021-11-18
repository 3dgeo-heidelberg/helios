FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /home
RUN apt update && apt install -y gcc g++ cmake make git wget unzip xz-utils && \
git clone -b dev https://github.com/3dgeo-heidelberg/helios.git

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
RUN ./configure -DCMAKE_INSTALL_PREFIX=. && make && make install

# Install Proj
WORKDIR /home/helios/lib
RUN apt install -y pkg-config libsqlite3-dev sqlite3 libtiff5-dev libcurl4-openssl-dev && \
mv proj-8.0.0 proj && cd proj && \
./configure && make -j 6 && make install

# Install GDAL
WORKDIR /home/helios/lib
RUN mv gdal-3.3.0 gdal && cd gdal && ./configure && make -j 6 && make install

# Install Boost
WORKDIR /home/helios/lib
RUN apt install -y libpython3.8-dev python3 python3-pip && \
mv boost_1_76_0 boost && cd boost && \
./bootstrap.sh --with-python=python3.8 && ./b2 cxxflags=-fPIC

# Set PYTHONPATH
ENV PYTHONPATH=/home/helios

# Compile Helios with PyBindings active
WORKDIR /home/helios
RUN cmake -DPYTHON_BINDING=1 -DPYTHON_VERSION=38 . && make -j 6

# Install PyHelios dependencies
RUN python3 -m pip install open3d

# Clean
WORKDIR /home/helios/lib
RUN rm *.tar.gz

WORKDIR /home/helios
