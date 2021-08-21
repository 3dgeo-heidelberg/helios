#!/bin/bash

# Author: Alberto M. Esmoris Pena
#
# Build the VTK library

# ---  PREPARE ENVIRONMENT  --- #
# ----------------------------- #
# Load environment if it has not been loaded yet
if [[ ${HELIOS_ENV} != 'ON' ]]; then
    this_dir=$(dirname ${BASH_SOURCE[0]})
    source ${this_dir}'/../local_env.sh'
fi


# ---  SCRIPT VARIABLES  --- #
# -------------------------- #
# VTK package
VTK_PKG='VTK-8.2.0.tar.gz'
# VTK URL
VTK_URL='https://www.vtk.org/files/release/8.2/'"${VTK_PKG}"
# VTK download path
VTK_TAR="${HELIOS_LIB_DIR}${VTK_PKG}"
# VTK lib path (where it is extracted)
VTK_DEF_DIR="${HELIOS_LIB_DIR}"$(sed 's/\.tar\.gz//g' <<< ${VTK_PKG})'/'
# VTK lib path (where it MUST be placed)
VTK_DIR="${HELIOS_LIB_DIR}"'vtk/'
# VTK lib install path (where it MUST be installed after build)
VTK_INSTALL_DIR="${VTK_DIR}"'install/'



# ---  SCRIPT LOGIC  --- #
# ---------------------- #
parse_build_args $@

# Download VTK if it is not downloaded yet
if [ ! -f "${VTK_TAR}" ]; then
    wget -c "${VTK_URL}" -O "${VTK_TAR}"
fi

# Decompress VTK if it has not been unzipped yet
if [ ! -d "${VTK_DIR}" ]; then
    cd "${HELIOS_LIB_DIR}"
    tar xzvf "${VTK_TAR}"
    mv "${VTK_DEF_DIR}" "${VTK_DIR}"
fi

# Build VTK
cd "${VTK_DIR}"
mkdir -p build
cd build
cmake \
    -DCMAKE_INSTALL_PREFIX:PATH="${VTK_INSTALL_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS:BOOL=ON \
    ..
make -j${HELIOS_BUILD_NCORES}
make -j${HELIOS_BUILD_NCORES} install
mv "${VTK_INSTALL_DIR}"'include/'*/* "${VTK_INSTALL_DIR}"'include/'

