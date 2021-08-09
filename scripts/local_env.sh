#!/bin/bash

# Author: Alberto M. Esmoris Pena
#
# Configure a local environment to easily work with helios++

# Warn helios environment is being loaded
echo 'Loading helios environment ...'

# Flag which specifies Helios environment has been loaded
HELIOS_ENV='ON'

# Path to the root directory of Helios++
HELIOS_DIR='/home/uadmin/git/helios/'
# Path to the scripts directory
HELIOS_SCRIPTS_DIR=${HELIOS_DIR}'scripts/'
# Path to the libraries directory
HELIOS_LIB_DIR=${HELIOS_DIR}'lib/'


# Warn helios environment has been loaded
echo 'Helios environment has been successfully loaded!'
