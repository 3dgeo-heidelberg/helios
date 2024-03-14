#!/bin/bash

# Author : Alberto M. Esmoris Pena
#
# Build the LAStools library

# ---  PREPARE ENVIRONMENT  --- #
# ----------------------------- #
# Load environment if it has not been loaded yet
if [[ ${HELIOS_ENV} != 'ON' ]]; then
    this_dir=$(dirname ${BASH_SOURCE[0]})
    source ${this_dir}'/../local_env.sh'
fi



# ---  SCRIPT VARIABLES  --- #
# -------------------------- #
# LAStools package
LAS_PKG='LAStools.zip'
# LAStools URL
LAS_URL='https://github.com/LAStools/LAStools/releases/download/v2.0.2/'"${LAS_PKG}"
# LAStools download path
LAS_ZIP="${HELIOS_LIB_DIR}${LAS_PKG}"
# LAStools lib path (where it MUST be placed)
LAS_DIR="${HELIOS_LIB_DIR}"'LAStools/'



# ---  SCRIPT LOGIC  --- #
# ---------------------- #
parse_build_args $@

# Download LAStools if it is not downloaded yet
if [ ! -f "${LAS_ZIP}" ]; then
    wget -c "${LAS_URL}" -O "${LAS_ZIP}"
fi

# Decompress LAStools if it has not been unzipped yet
if [ ! -d "${LAS_DIR}" ]; then
    cd "${HELIOS_LIB_DIR}"
    unzip "${LAS_ZIP}"
fi

# Build LAStools
cd "${LAS_DIR}"
cmake . -DCMAKE_INSTALL_PREFIX:PATH="${LAS_DIR}"
make -j${HELIOS_BUILD_NCORES}
