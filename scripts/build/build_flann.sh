#!/bin/bash

# Author : Alberto M. Esmoris Pena
#
# Build the FLANN library

# ---  PREPARE ENVIRONMENT  --- #
# ----------------------------- #
# Load environment if it has not been loaded yet
if [[ ${HELIOS_ENV} != 'ON' ]]; then
    this_dir=$(dirname ${BASH_SOURCE[0]})
    source ${this_dir}'/../local_env.sh'
fi



# ---  SCRIPT VARIABLES  --- #
# -------------------------- #
# FLANN version
FLANN_VER='1.9.1'
# FLANN package
FLANN_PKG="${FLANN_VER}"'.tar.gz'
# FLANN URL
FLANN_URL='https://github.com/mariusmuja/flann/archive/'"${FLANN_PKG}"
# FLANN download path
FLANN_TAR="${HELIOS_LIB_DIR}${FLANN_PKG}"
# FLANN lib path (where it MUST be placed)
FLANN_DIR="${HELIOS_LIB_DIR}"'flann/'



# ---  SCRIPT LOGIC  --- #
# ---------------------- #
parse_build_args $@

# Download FLANN if it is not downloaded yet
if [ ! -f "${FLANN_TAR}" ]; then
    wget -c "${FLANN_URL}" -O "${FLANN_TAR}"
fi

# Decompress FLANN if it has not been unzipped yet
if [ ! -d "${FLANN_DIR}" ]; then
    cd "${HELIOS_LIB_DIR}"
    tar xzvf "${FLANN_TAR}"
fi
mv 'flann-'"${FLANN_VER}" "${FLANN_DIR}"

# Change directory to FLANN
cd "${FLANN_DIR}"

# Fix empty source compilation (Just a workaround)
touch "src/cpp/empty.cpp"
XYZ='add_library(flann_cpp\ SHARED\ \"empty.cpp\")'
ABC='cuda_add_library(flann_cuda\ SHARED\ \"empty.cpp\")'
UVW='add_library(flann\ SHARED\ \"empty.cpp\")'
sed -e "s/add_library(flann_cpp\ SHARED\ \"\")/${XYZ}/g"\
    -e "s/cuda_add_library(flann_cuda\ SHARED\ \"\")/${ABC}/g"\
    -e "s/add_library(flann\ SHARED\ \"\")/${UVW}/g"\
    -i src/cpp/CMakeLists.txt 

# Build FLANN
mkdir -p build install
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install ..
make -j${HELIOS_BUILD_NCORES}
make install -j${HELIOS_BUILD_NCORES}

