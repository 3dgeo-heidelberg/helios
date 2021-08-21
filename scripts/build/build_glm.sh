#!/bin/bash

# Author: Alberto M. Esmoris Pena
#
# Build the GLM library.
# In fact not build, but place adequately (header only library)

# ---  PREPARE ENVIRONMENT  --- #
# ----------------------------- #
# Load environment if it has not been loaded yet
if [[ ${HELIOS_ENV} != 'ON' ]]; then
    this_dir=$(dirname ${BASH_SOURCE[0]})
    source ${this_dir}'/../local_env.sh'
fi



# ---  SCRIPT VARIABLES  --- #
# -------------------------- #
# GLM version
GLM_VER='0.9.9.8'
# GLM package
GLM_PKG='glm-'"${GLM_VER}"'.zip'
# GLM URL
GLM_URL='https://github.com/g-truc/glm/releases/download/'
GLM_URL="${GLM_URL}${GLM_VER}/${GLM_PKG}"
# GLM download path
GLM_TAR="${HELIOS_LIB_DIR}${GLM_PKG}"
# GLM lib path (where it MUST be placed)
GLM_DIR="${HELIOS_LIB_DIR}"'glm/'


# ---  SCRIPT LOGIC  --- #
# ---------------------- #
parse_build_args $@

# Download GLM if it is not downloaded yet
if [ ! -f "${GLM_TAR}" ]; then
    wget -c "${GLM_URL}" -O "${GLM_TAR}"
fi

# Decompress GLM  if it has not been unziped yet
if [ ! -d "${GLM_DIR}" ]; then
    cd "${HELIOS_LIB_DIR}"
    unzip "${GLM_TAR}"
fi

