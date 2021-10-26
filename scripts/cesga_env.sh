#!/bin/bash

# Author: Alberto M. Esmoris Pena
#
# Configure a local environment to easily work with Helios++ in CESGA

echo 'Loading Helios CESGA environment ...'

# Flag which specifies Helios environment has been loaded
export HELIOS_ENV='ON'

# Path to the root directory of Helios++
export HELIOS_DIR="${HOME}"'/git/helios/'
# Path to the scripts directory
export HELIOS_SCRIPTS_DIR=${HELIOS_DIR}'scripts/'
# Path to the libraries directory
export HELIOS_LIB_DIR=${HELIOS_DIR}'lib/'

# Load utils
source "${HELIOS_SCRIPTS_DIR}"'build/build_utils.sh'

# Load Helios CESGA modules
HC_MODULES=(
    'python/3.8.1'
    'git/2.24.0'
    'cmake/3.12.4'
    'libtiff/4.1.0'
    'eigen/3.3.7'
    'lz4/1.8.3'
    'qhull/2019.1'
    'boost/1.74.0-python-3.8.1'
)
for mod in ${HC_MODULES[@]}; do
    cmd="module load ${mod}"
    echo '$'${cmd}
    ${cmd}
    echo -ne "\n"
done

# Configure LD library path
export LD_LIBRARY_PATH=\
"${LD_LIBRARY_PATH}:${HELIOS_LIB_DIR}lapack/install/lib/"
# Path to helios binary
export HELIOS_BIN=${HELIOS_DIR}'helios'
# Helios CESGA script
export HELIOS_CESGA="${HELIOS_SCRIPTS_DIR}run/helios_cesga.sh"

# Warn Helios environment has been loaded
echo 'Helios CESGA environment has been successfully loaded!'

