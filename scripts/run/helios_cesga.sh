#!/bin/bash

# Author: Alberto M. Esmoris Pena
#
# Run Helios++ in CESGA

# ---  PREPARE ENVIRONMENT  --- #
# ----------------------------- #
# Load environment if it has not been loaded yet
if [[ ${HELIOS_ENV} != 'ON' ]]; then
    this_dir=$(dirname ${BASH_SOURCE[0]})
    source ${this_dir}'/../cesga_env.sh'
fi

${HELIOS_BIN} $@
