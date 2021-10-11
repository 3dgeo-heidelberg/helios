#!/bin/bash

# Author: Alberto M. Esmoris Pena
#
# Build the armadillo library at CESGA FT2

# ---  PREPARE ENVIRONMENT  --- #
# ----------------------------- #
# Load environment if it has not been loaded yet
if [[ ${HELIOS_ENV} != 'ON' ]]; then
    this_dir=$(dirname ${BASH_SOURCE[0]})
    source ${this_dir}'/../local_env.sh'
fi



# ---  SCRIPT VARIABLES  --- #
# -------------------------- #
# Armadillo package
ARMA_PKG='armadillo-10.6.1.tar.xz'
# Armadillo URL
ARMA_URL='https://altushost-swe.dl.sourceforge.net/project/arma/'"${ARMA_PKG}"
# Armadillo download path
ARMA_TAR="${HELIOS_LIB_DIR}${ARMA_PKG}"
# Armadillo lib path (where it is extracted)
ARMA_DEF_DIR="${HELIOS_LIB_DIR}"$(sed 's/\.tar\.xz//g' <<< ${ARMA_PKG})'/'
# Armadillo lib path (where it MUST be placed)
ARMA_DIR="${HELIOS_LIB_DIR}"'armadillo/'



# ---  SCRIPT LOGIC  --- #
# ---------------------- #
parse_build_args $@

# Download armadillo if it is not downloaded yet
if [ ! -f "${ARMA_TAR}" ]; then
    wget -c "${ARMA_URL}" -O "${ARMA_TAR}" --no-check-certificate
fi

# Decompress armadillo if it has not been unzipped yet
if [ ! -d "${ARMA_DIR}" ]; then
    cd "${HELIOS_LIB_DIR}"
    tar xvf "${ARMA_TAR}"
    mv "${ARMA_DEF_DIR}" "${ARMA_DIR}"
fi

# Build armadillo
cd "${ARMA_DIR}"
LAP_LIB="${HELIOS_LIB_DIR}"'lapack/install/lib/liblapack.so'
cmake . -DCMAKE_INSTALL_PREFIX:PATH="${ARMA_DIR}" \
    -DLAPACK_LIBRARY="${LAP_LIB}"
make -j${HELIOS_BUILD_NCORES} install

