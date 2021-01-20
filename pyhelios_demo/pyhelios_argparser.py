#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import sys

parser = argparse.ArgumentParser(description='Parser for PyHelios')

parser.add_argument('survey_file', help='Absolute or relative path to .XML survey file to be used for simulation.')

parser.add_argument('--test', dest='test_is_desired', action='store_const', const=True,
                    help='Run test to check whether PyHelios is importing correctly. '
                         'Also returns current PyHelios version.')

parser.add_argument('--assets', dest='assets_path', default='assets/',
                    help='Specify the path to assets directory. Default: "assets/".')

parser.add_argument('--output', dest='output_path', default='output/',
                    help='Specify the path to assets directory. Default: "output/".')

parser.add_argument('--writeWaveform', dest='write_waveform_flag', action='store_const', const=True, default=False,
                    help='Use this flag to enable full waveform writing. '
                         'By default waveform is NOT written to output file.')

parser.add_argument('--calcEchowidth', dest='calc_echowidth_flag', action='store_const', const=True, default=False,
                    help='Use this flag to enable full waveform fitting.'
                         'By default the full waveform is NOT fitted.')

parser.add_argument('--fullwaveNoise', dest='fullwave_noise_flag', action='store_const', const=True, default=False,
                    help='Use this flag to add noise when computing full waveform.'
                         'By default full waveform noise is disabled.')

parser.add_argument('--seed', dest='randomness_seed',
                    help='Specify the seed for randomness generation. '
                         'It can be an intenger, a decimal or a timestamp with format YYYY-mm-DD HH::MM::SS. '
                         'By default: a random seed is generated.')

parser.add_argument('--lasOutput', dest='las_output_flag', action='store_const', const=True, default=False,
                    help='Use this flag to generate the output point cloud in LAS format.')

parser.add_argument('--zipOutput', dest='zip_output_flag', action='store_const', const=True, default=False,
                    help='Use this flag to generate compressed output.')

parser.add_argument('-j', '--njobs', '--nthreads', type=int, dest='number_of_threads', default=0,
                    help='Specify the number of jobs used to compute the simulation. '
                         'By default: all available threads are used.')

parser.add_argument('--rebuildScene', dest='rebuild_scene_flag', action='store_const', const=True, default=False,
                    help='Force scene rebuild even when a previously built scene is available.'
                         'By default: previous scene is used if found.')

parser.add_argument('--disablePlatformNoise', dest='platform_noise_disabled_flag', action='store_const', const=True,
                    default=False,
                    help='Disable leg noise, no matter what is specified on XML files. '
                         'By default: XML specifications are considered.')

parser.add_argument('--disableLegNoise', dest='leg_noise_disabled_flag', action='store_const', const=True,
                    default=False,
                    help='Disable platform noise, no matter what is specified on XML files. '
                         'By default: XML specifications are considered.')

parser.add_argument('--silent', dest='loggingsilent', action='store_const', const=True,
                    help='Disable logging output. By default: only information and errors are reported.')

parser.add_argument('-q', '--quiet', dest='loggingquiet', action='store_const', const=True,
                    help='Specify the verbosity level to errors only. '
                         'By default: only information and errors are reported.')

parser.add_argument('-v', dest='loggingv', action='store_const', const=True,
                    help='Specify the verbosity level to errors, information and warnings. '
                    'By default: only information and errors are reported.')

parser.add_argument('-v2', '-vv', dest='loggingv2', action='store_const', const=True,
                    help='Specify the verbosity level to report all messages. '
                    'By default: only information and errors are reported.')

parser.add_argument('-lp', '--livetrajectoryplot', dest='live_trajectory_plot', action='store_const', const=True,
                    help='Enable live matplotlib plotting of scanner trajectory.')

parser.add_argument('-ps', '--polyscope', dest='plot_result', action='store_const', const=True,
                    help='Display scanner trajectory and resulting data as a point cloud in polyscope.'
                         'Requires Polyscope module to be installed in environment.')

parser.add_argument('-o3d', '--open3d', dest='open3d', action='store_const', const=True,
                    help='Enable live open3d plot of simulation.')

args = parser.parse_args()


if __name__ == '__main__':
    # Change working directory to helios path and configure runpath within pyhelios.
    helios_run_path = r'D:\Documents\ss_2020\helios\202007_helios_bin\run/'

    # Survey to be used.
    survey_path = 'toyblocks/custom_als_toyblocks.xml'

    # Add run path to python path.
    sys.path.append(helios_run_path)

    # PyHelios import only now possible.
    import pyhelios