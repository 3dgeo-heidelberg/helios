#!/bin/bash

# Author: Alberto M. Esmoris Pena
#
# Build the GDAL library

# ---  PREPARE ENVIRONMENT  --- #
# ----------------------------- #
# Load environment if it has not been loaded yet

if [[ ${HELIOS_ENV} != 'ON' ]]; then
    this_dir=$(dirname ${BASH_SOURCE[0]})
    source ${this_dir}'/../local_env.sh'
fi



# ---  SCRIPT VARIABLES  --- #
# -------------------------- #
# GDAL version
GDAL_VER='3.3.0'
# GDAL package
GDAL_PKG='gdal-3.3.0.tar.gz'
# GDAL URL
GDAL_URL='https://github.com/OSGeo/gdal/releases/download/v'
GDAL_URL="${GDAL_URL}${GDAL_VER}/${GDAL_PKG}"
# GDAL download path
GDAL_TAR="${HELIOS_LIB_DIR}${GDAL_PKG}"
# GDAL lib path (where it is extracted)
GDAL_DEF_DIR="${HELIOS_LIB_DIR}"$(sed 's/\.tar\.gz//g' <<< ${GDAL_PKG})'/'
# GDAL lib path (where it MUST be placed)
GDAL_DIR="${HELIOS_LIB_DIR}"'gdal/'
# PROJ package
PROJ_PKG='proj-8.0.0.tar.gz'
# PROJ URL
PROJ_URL='http://download.osgeo.org/proj/'"${PROJ_PKG}"
# PROJ download path
PROJ_TAR="${HELIOS_LIB_DIR}${PROJ_PKG}"
# PROJ lib path (where it is extracted)
PROJ_DEF_DIR="${HELIOS_LIB_DIR}"$(sed 's/\.tar\.gz//g' <<< ${PROJ_PKG})
# PROJ lib path (where it MUST be placed)
PROJ_DIR="${GDAL_DIR}"'proj/'
# PROJ build dir (where built proj MUST be placed)
PROJ_BDIR="${GDAL_DIR}"'projlib/'

# ---  SCRIPT LOGIC  --- #
# ---------------------- #
# Download GDAL if it is not downloaded yet
if [ ! -f "${GDAL_TAR}" ]; then
    wget -c "${GDAL_URL}" -O "${GDAL_TAR}"
fi

# Download PROJ if it is not downloaded yet
if [ ! -f "${PROJ_TAR}" ]; then
    wget -c "${PROJ_URL}" -O "${PROJ_TAR}"
fi

# Decompress GDAL if it has not been unzipped yet
if [ ! -d "${GDAL_DIR}" ]; then
    cd "${HELIOS_LIB_DIR}"
    tar xvf "${GDAL_TAR}"
    mv "${GDAL_DEF_DIR}" "${GDAL_DIR}"
fi

# Decompress PROJ if it has not been unzipped yet
if [ ! -d "${PROJ_DIR}" ]; then
    cd "${HELIOS_LIB_DIR}"
    tar xvf "${PROJ_TAR}"
    mv "${PROJ_DEF_DIR}" "${PROJ_DIR}"
fi

# Build PROJ
cd "${PROJ_DIR}"
./configure --prefix="${PROJ_BDIR}"
make
make install

# Build GDAL
cd "${GDAL_DIR}"
./configure --prefix="${GDAL_DIR}" --with-proj="${PROJ_BDIR}"
make
make install

