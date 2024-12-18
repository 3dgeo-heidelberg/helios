#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse

parser = argparse.ArgumentParser(description='Parser for PyHelios')

parser.add_argument('survey_file', help='Absolute or relative path to .XML survey file to be used for simulation.')

# tds specific arguments
parser.add_argument('--interval_s', dest='time_interval_s', default=30,
                    help='Specify the fix time interval of each survey subset.')
parser.add_argument('--deleteAfter', dest='delete_flag', action='store_const', const=True, default=False,
                    help='Use this flag to delete all intermediate files after simulation.')

# general helios arguments
parser.add_argument('--assets', dest='assets_path', default='assets/',
                    help='Specify the path to assets directory. Default: "assets/".')

parser.add_argument('--output', dest='output_path', default='output/',
                    help='Specify the path to assets directory. Default: "output/".')

parser.add_argument('--zipOutput', dest='zip_output_flag', action='store_const', const=True, default=False,
                    help='Use this flag to generate compressed output.')

parser.add_argument('--primitiveThreshold', dest='prim_threshold', default=0.6,
                    help='Specify the maximum primitiv percentage per interval.')
parser.add_argument('--minInterval', dest='min_int_t', default=0.5,
                    help='Minimum interval length, no further splitting with intervals smaller than the set value.')
parser.add_argument('--objInterest',dest='obj_of_int', nargs='+', type=int, default=None,
                    help='List of objects to be scanned, intervals without any of these objects wont be scanned')
parser.add_argument('--tileSize', dest='tilesize', default=30,
                    help='Tilesize of each sub xyz tile.')
args = parser.parse_args()
