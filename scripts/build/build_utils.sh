#!/bin/bash

# Author : Alberto M. Esmoris Pena
#
# Build utils

#  ---  DEFAULTS  ---  #
#  ------------------  #
# Python version that boost must link against
export BOOST_PYTHON_TARGET_VERSION='python3.7'
# Enable visualization (ON) or disable it (OFF)
export PCL_VISUALIZATION='OFF'
# Specify the helios point cloud library must be built or not (default no)
export HELIOS_BUILD_PCL=''
# Specify how many cores use when building helios libraries
export HELIOS_BUILD_NCORES=4

# ---  FUNCTIONS  --- #
# ------------------- #
function parse_build_args_help {
    cat << EOF

Helios building scripts support following arguments:


    -p <version> : Specify the python version to be used
        Example: -p 3.8

    -c : Enable PCL compilation.
        NOTICE by default PCL is enabled without visualization.
        Visualization can be enabled through -v argument

    -v : Enable PCL visualization module which implies building VTK

    -w <num workers> : Specify how many workers (cores) use to compile
        Example: -w 6

    -h : Show this help

EOF
}
export -f parse_build_args_help

function parse_build_args {
    # Overwrite defaults by user argumments
    while [[ -n "$1" ]]; do
        case "$1" in
            -p) export BOOST_PYTHON_TARGET_VERSION='python'"$2"
                echo    'BOOST_PYTHON_TARGET_VERSION set to' \
                        "${BOOST_PYTHON_TARGET_VERSION}"
                shift ;;
            -v) export PCL_VISUALIZATION='ON'
                echo 'PCL_VISUALIZATION set to '"${PCL_VISUALIZATION}"
                ;;
            -c) export HELIOS_BUILD_PCL=1
                echo 'Building Helios with PCL has been enabled'
                ;;
            -w) export HELIOS_BUILD_NCORES=$2
                echo 'Building Helios with '"${HELIOS_BUILD_NCORES}"' workers'
                shift ;;
            -h) parse_build_args_help
                exit ;;
        esac
        shift
    done
}
export -f parse_build_args
