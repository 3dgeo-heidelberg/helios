#!/bin/bash

# ########################################################################
#                                                                        #
# AUTHOR : Alberto M. Esmorís Pena                                       #
#                                                                        #
# Script to transform an input CSV file, which rows are pulse records    #
# written by the HDA_PulseRecorder component of HELIOS++, to a LAS/LAZ   #
# file.                                                                  #
#                                                                        #
# ########################################################################

# Path or alias to TXT2LAS util
TXT2LAS='txt2las'

# Function to show help
function show_help {
    cat << EOF

hda_pulse_calc_intensity_csv_to_laz.sh

    Argument 1: Path to the input CSV file

    Argument 2: Path to the output LAZ file


EOF
}

# Handle input arguments
if [[ $# -ne 2 ]]; then
    echo -e 'ERROR: Exactly two arguments must be provided\n'
    show_help
    exit 1
fi


echo -e "Transforming \"$1\" to \"$2\" ..."
mkdir -p $(dirname "$2")
${TXT2LAS} -i "$1" \
    -set_version 1.4 -parse xyz01234567 -rescale 1e-5 1e-5 1e-5 \
    -add_attribute 10 'incidence_angle_rad' 'incidence_angle_rad' \
    -add_attribute 10 'target_range_m' 'target_range_m' \
    -add_attribute 10 'target_area_m2' 'target_area_m2' \
    -add_attribute 10 'radius_m' 'radius_m' \
    -add_attribute 10 'bdrf' 'bdrf' \
    -add_attribute 10 'cross_section_m2' 'cross_section_m2' \
    -add_attribute 10 'received_power' 'received_power' \
    -add_attribute 2 'captured' 'captured' \
    -o "$2"


