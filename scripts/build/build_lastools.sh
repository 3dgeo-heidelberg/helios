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
LAS_URL='https://lastools.github.io/download/'"${LAS_PKG}"
# LAStools download path
LAS_TAR="${HELIOS_LIB_DIR}${LAS_PKG}"
# LAStools lib papth (where it MUST be placed)
LAS_DIR="${HELIOS_LIB_DIR}"'LAStools/'



# ---  SCRIPT LOGIC  --- #
# ---------------------- #
# Download LAStools if it is not downloaded yet
if [ ! -f "${LAS_TAR}" ]; then
    wget -c "${LAS_URL}" -O "${LAS_TAR}"
fi

# Decompress LAStools if it has not been unzipped yet
if [ ! -d "${LAS_DIR}" ]; then
    cd "${HELIOS_LIB_DIR}"
    unzip "${LAS_TAR}"
fi

# Build LAStools
cd "${LAS_DIR}"
cmake . -DCMAKE_INSTALL_PREFIX:PATH="${LAS_DIR}"
make
