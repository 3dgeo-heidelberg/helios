#!/bin/bash

# Author : Alberto M. Esmoris Pena
#
# Build the boost library

# ---  PREPARE ENVIRONMENT  --- #
# ----------------------------- #
# Load environment if it has not been loaded yet
if [[ ${HELIOS_ENV} != 'ON' ]]; then
    this_dir=$(dirname ${BASH_SOURCE[0]})
    source ${this_dir}'/../local_env.sh'
fi



# ---  SCRIPT VARIABLES  --- #
# -------------------------- #
# Boost package
BOOST_PKG='boost_1_76_0.tar.gz'
# Boost URL
BOOST_URL='https://boostorg.jfrog.io/artifactory/main/release/1.76.0/source/'
BOOST_URL="${BOOST_URL}${BOOST_PKG}"
# Boost download path
BOOST_TAR="${HELIOS_LIB_DIR}${BOOST_PKG}"
# Boost lib path (where it is extracted)
BOOST_DEF_DIR="${HELIOS_LIB_DIR}"$(sed 's/\.tar\.gz//g' <<< ${BOOST_PKG})
# Boost lib path (where it MUST be placed)
BOOST_DIR="${HELIOS_LIB_DIR}"'boost/'



# ---  SCRIPT LOGIC  --- #
# ---------------------- #
parse_build_args $@

# Download armadillo if it is not downloaded yet
if [ ! -f "${BOOST_TAR}" ]; then
    wget -c "${BOOST_URL}" -O "${BOOST_TAR}"
fi

# Decompress boost if it has not been unzipped yet
if [ ! -d "${BOOST_DIR}" ]; then
    cd "${HELIOS_LIB_DIR}"
    tar xzvf "${BOOST_TAR}"
    mv ${BOOST_DEF_DIR} ${BOOST_DIR}
fi

# Build boost
cd "${BOOST_DIR}"
./bootstrap.sh --with-python=${BOOST_PYTHON_TARGET_VERSION}  --prefix="${BOOST_DIR}"
./b2 -j ${HELIOS_BUILD_NCORES} cxxflags=-fPIC
./b2 -j ${HELIOS_BUILD_NCORES} install
