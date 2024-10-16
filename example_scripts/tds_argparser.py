#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse

parser = argparse.ArgumentParser(description='Parser for PyHelios')

parser.add_argument('survey_file', help='Absolute or relative path to .XML survey file to be used for simulation.')

# tds specific arguments
parser.add_argument('--interval_s', dest='time_interval_s', default=30,
                    help='Specify the fix time interval of each survey subset.')

# general helios arguments
parser.add_argument('--assets', dest='assets_path', default='assets/',
                    help='Specify the path to assets directory. Default: "assets/".')

parser.add_argument('--output', dest='output_path', default='output/',
                    help='Specify the path to assets directory. Default: "output/".')

parser.add_argument('--lasOutput', dest='las_output_flag', action='store_const', const=True, default=False,
                    help='Use this flag to generate the output point cloud in LAS v1.4 format.')

parser.add_argument('--zipOutput', dest='zip_output_flag', action='store_const', const=True, default=False,
                    help='Use this flag to generate compressed output.')

args = parser.parse_args()