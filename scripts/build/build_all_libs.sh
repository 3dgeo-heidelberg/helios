#!/bin/bash

# Author: Alberto M. Esmoris PEna
#
# Build all libraries required by Helios++

# ---  PREPARE ENVIRONMENT  --- #
# ----------------------------- #
# Load environment if it has not been loaded yet
if [[ ${HELIOS_ENV} != 'ON' ]]; then
    this_dir=$(dirname ${BASH_SOURCE[0]})
    source ${this_dir}'/../local_env.sh'
fi



# ---  SCRIPT LOGIC  --- #
# ---------------------- #
${HELIOS_SCRIPTS_DIR}/build/build_armadillo.sh
${HELIOS_SCRIPTS_DIR}/build/build_boost.sh
${HELIOS_SCRIPTS_DIR}/build/build_glm.sh
${HELIOS_SCRIPTS_DIR}/build/build_lastools.sh
${HELIOS_SCRIPTS_DIR}/build/build_gdal.sh
