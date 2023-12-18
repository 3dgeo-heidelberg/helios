#!/bin/bash

# Author: Alberto M. Esmoris PEna
#
# Build all libraries required by Helios++ in CESGA
#
# Calling with following command is recommended:
#   ./build_cesga.sh -p 3.8 -w 12 -c
# To compile later use:
#   cmake -DPYTHON_BINDING=1 -DPCL_BINDING=0 -DBOOST_DYNAMIC_LIBS=1 \
#       -DPYTHON_VERSION=38 -DLAPACK_LIB="lib/lapack/install/lib/liblapack.so"
# And:
#   make -j12
# To run tests and check everything is working:
#   LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:lib/lapack/install/lib/ ./helios --test
#       -DPYTHON_VERSION=38 -DLAPACK_LIB="${HELIOS_LIB_DIR}/lapack/install/lib/liblapack.so"
# And:
#   make -j12
# To run tests and check everything is working:
#   LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${HELIOS_LIB_DIR}/lapack/install/lib/" ./helios --test
# Alternatively, it is possible to use:
#   scripts/run/helios_cesga --test --testDir ${HELIOS_DIR}data/test/
# Also, if environment is loaded:
#   ${HELIOS_CESGA} --test --testDir ${HELIOS_DIR}data/test/

# ---  PREPARE ENVIRONMENT  --- #
# ----------------------------- #
# Load environment if it has not been loaded yet
if [[ ${HELIOS_ENV} != 'ON' ]]; then
    this_dir=$(dirname ${BASH_SOURCE[0]})
    source ${this_dir}'/../cesga_env.sh'
fi


# ---  SCRIPT LOGIC  --- #
# ---------------------- #
parse_build_args $@

${HELIOS_SCRIPTS_DIR}/build/build_lapack.sh
${HELIOS_SCRIPTS_DIR}/build/build_armadillo_cesga.sh
${HELIOS_SCRIPTS_DIR}/build/build_glm.sh
${HELIOS_SCRIPTS_DIR}/build/build_lastools.sh
${HELIOS_SCRIPTS_DIR}/build/build_gdal.sh
if [[ ${HELIOS_BUILD_PCL} ]]; then
    if [[ ${PCL_VISUALIZATION} == 'ON' ]]; then
        ${HELIOS_SCRIPTS_DIR}/build/build_vtk.sh
    fi
    ${HELIOS_SCRIPTS_DIR}/build/build_flann.sh
    ${HELIOS_SCRIPTS_DIR}/build/build_pcl_cesga.sh
fi
