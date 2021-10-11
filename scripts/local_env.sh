#!/bin/bash

# Author: Alberto M. Esmoris Pena
#
# Configure a local environment to easily work with Helios++

# Warn helios environment is being loaded
echo 'Loading Helios environment ...'

# Flag which specifies Helios environment has been loaded
export HELIOS_ENV='ON'

# Path to the root directory of Helios++
export HELIOS_DIR='/home/uadmin/git/helios/'
# Path to the scripts directory
export HELIOS_SCRIPTS_DIR=${HELIOS_DIR}'scripts/'
# Path to the libraries directory
export HELIOS_LIB_DIR=${HELIOS_DIR}'lib/'


# Load utils
source "${HELIOS_SCRIPTS_DIR}"'build/build_utils.sh'

# Warn Helios environment has been loaded
echo 'Helios environment has been successfully loaded!'
