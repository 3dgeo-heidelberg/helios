#!/bin/bash

# Author: Alberto M. Esmoris Pena
#
# Build the Point Cloud Library (PCL)

# ---  PREPARE ENVIRONMENT  --- #
# ----------------------------- #
# Load environment if it has not been loaded yet
if [[ ${HELIOS_ENV} != 'ON' ]]; then
    this_dir=$(dirname ${BASH_SOURCE[0]})
    source ${this_dir}'/../local_env.sh'
fi



# ---  SCRIPT VARIABLES  --- #
# -------------------------- #
# PCL package
PCL_PKG='pcl-1.13.0.tar.gz'
# PCL URL
PCL_URL='https://github.com/PointCloudLibrary/pcl/archive/refs/tags/'"${PCL_PKG}"
# PCL download path
PCL_TAR="${HELIOS_LIB_DIR}${PCL_PKG}"
# PCL lib path (where it is extracted)
PCL_DEF_DIR="${HELIOS_LIB_DIR}pcl-"$(sed 's/\.tar\.gz//g' <<< ${PCL_PKG})'/'
# PCL lib path (where it MUST be placed)
PCL_DIR="${HELIOS_LIB_DIR}"'pcl/'
PCL_INSTALL_DIR="${PCL_DIR}"'install/'
# Directory to the VTK for visualization
PCL_VTK_DIR=$(ls "${HELIOS_LIB_DIR}vtk/install/lib/cmake" | grep vtk)
PCL_VTK_DIR="${HELIOS_LIB_DIR}vtk/install/lib/cmake/${PCL_VTK_DIR}/"



# ---  SCRIPT LOGIC  --- #
# ---------------------- #
parse_build_args $@

# Download PCL if it is not downloaded yet
if [ ! -f "${PCL_TAR}" ]; then
    wget -c "${PCL_URL}" -O "${PCL_TAR}"
fi

# Decompress PCL if it has not been unzipped yet
if [ ! -d "${PCL_DIR}" ]; then
    cd "${HELIOS_LIB_DIR}"
    tar xzvf "${PCL_TAR}"
    mv "${PCL_DEF_DIR}" "${PCL_DIR}"
fi

# Build PCL
cd "${PCL_DIR}"
mkdir -p build
cd build
cmake \
    -DCMAKE_INSTALL_PREFIX:PATH="${PCL_INSTALL_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBOOST_ROOT="${HELIOS_LIB_DIR}"'boost/' \
    -DBUILD_visualization=${PCL_VISUALIZATION} \
    -DVTK_DIR="${PCL_VTK_DIR}" \
    -DCMAKE_INCLUDE_PATH="${HELIOS_LIB_DIR}"'vtk/install/include' \
    ..
make -j${HELIOS_BUILD_NCORES}
make -j${HELIOS_BUILD_NCORES} install
mv "${PCL_INSTALL_DIR}"'include/'*/* "${PCL_INSTALL_DIR}"'include/'

