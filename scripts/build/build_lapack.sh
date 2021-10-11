#!/bin/bash

# Author: Alberto M. Esmoris Pena
# Brief: Handle LAPACK install on CESGA FT2 so Armadillo can be fully built

# ---  PREPARE ENVIRONMENT  --- #
# ----------------------------- #
# Load environment if it has not been loaded yet
if [[ ${HELIOS_ENV} != 'ON' ]]; then
    this_dir=$(dirname ${BASH_SOURCE[0]})
    source ${this_dir}'/../local_env.sh'
fi



# ---  SCRIPT VARIABLES  --- #
# -------------------------- #
# LAPACK package
LAP_PKG='v3.9.0.tar.gz'
# LAPACK URL
LAP_URL='https://github.com/Reference-LAPACK/lapack/archive/'"${LAP_PKG}"
# LAPACK download path
LAP_TAR="${HELIOS_LIB_DIR}${LAP_PKG}"
# LAPACK lib path (where it MUST be placed)
LAP_DIR="${HELIOS_LIB_DIR}"'lapack/'

# ---  SCRIPT LOGIC  --- #
# ---------------------- #
parse_build_args $@

# Download LAPACK if it is not downloaded yet
if [ ! -f "${LAP_TAR}" ]; then
    wget -c "${LAP_URL}" -O "${LAP_TAR}" --no-check-certificate
fi

# Decompress LAPACK if it has not been unzipped yet
if [ ! -d "${LAP_DIR}" ]; then
    cd "${HELIOS_LIB_DIR}"
    tar xvf "${LAP_TAR}"
    mv lapack-*/ ${LAP_DIR}
fi

# Build LAPACK
cd "${LAP_DIR}"
mkdir -p build install
cd build
cmake -DCMAKE_INSTALL_PREFIX="${LAP_DIR}install" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    ..
make -j${HELIOS_BUILD_NCORES}
make -j${HELIOS_BUILD_NCORES} install

# Workaround for lib64
mv "${LAP_DIR}install/lib64" "${LAP_DIR}install/lib"
