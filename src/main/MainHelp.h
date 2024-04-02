#pragma once

#include <iostream>

namespace helios { namespace main{

/**
 * @brief Output the main help of HELIOS++ through standard output
 */
void printMainHelp(){
    std::cout << "HELIOS++ main help:\n\n"
    <<  "\tSyntax: helios <survey_file_path> [OPTIONAL ARGUMENTS]\n\n"
    <<  "\tOPTIONAL ARGUMENTS:\n\n"

    <<  "\t\t-h or --help : Show this help\n\n"

    << "\t\t--version : Show the full version of HELIOS++\n\n"

    <<  "\t\t--test : Run tests to check helios++ behaves as expected\n\n"

    <<  "\t\t--unzip <input_path> <output_path>\n"
    <<  "\t\t\tDecompress the file at input path and write it decompressed "
    <<  "at output path.\n"
    <<  "\t\t\tFile at input path must be the compressed output of helios++"
    <<  "\n\n"

    <<  "\t\t--assets <dir_path> : Specify the path to assets directory\n"
    <<  "\t\t\tBy default: ./assets/\n\n"

    <<  "\t\t--output <dir_path> : Specify the path to output directory\n"
    <<  "\t\t\tBy default: ./output/\n\n"

    << "\t\t--splitByChannel : Use this flag to enable the one-file-per-device"
       "\n\t\t\twriting mode when using a multi-channel scanner.\n"
       "\t\t\tBy default one-file-for-all writing is enabled\n\n"

    <<  "\t\t--writeWaveform : Use this flag to enable full waveform "
    <<  "writing\n"
    <<  "\t\t\tBy default waveform is NOT written to output file\n\n"

    << "\t\t--writePulse : Use this flag to enable pulse-wise data writing\n"
    << "\t\t\tBy default pulse-wise ata is NOT written to output file\n\n"

    <<  "\t\t--calcEchowidth : Use this flag to enable full waveform "
    <<  "fitting\n"
    <<  "\t\t\tBy default the full waveform is NOT fitted\n\n"

    <<  "\t\t--fullwaveNoise : Use this flag to add noise when computing "
    <<  "full waveform\n"
    <<  "\t\t\tBy default: full waveform noise is disabled\n\n"

    <<  "\t\t--fixedIncidenceAngle : Use this flag to use fixed incidence "
    <<  "angle\n"
    <<  "\t\t\tFixed incidence angle of exactly 0.0 will be considered for "
    <<  "all intersections\n\n"

    <<  "\t\t--seed <seed>: Specify the seed for randomness generation\n"
    <<  "\t\t\tIt can be an intenger, a decimal or a timestamp with format "
    <<  "\n\t\t\tYYYY-mm-DD HH::MM::SS\n"
    <<  "\t\t\t\tBy default: a random seed is generated\n\n"

    <<  "\t\t--gpsStartTime <string>: Specify a fixed start time for GPS\n"
    <<  "\t\t\tIt can be either a posix timestamp or a"
        "\"YYYY-MM-DD hh:mm:ss\" date time string\n"
    <<  "\t\t\t\tBy default: An empty string \"\" is used, which leads to\n"
    <<  "\t\t\t\t\tusing current system time\n\n"

    <<  "\t\t--lasOutput : Use this flag to generate the output point cloud "
        "in LAS format (v 1.4)\n\n"
    <<  "\t\t--las10: Use this flag to write in LAS format (v 1.0)\n\n"

    <<  "\t\t--zipOutput : Use this flag to generate compressed output\n\n"

    <<  "\t\t--lasScale : Specify the decimal scale factor for LAS output"
    <<  "\n\n"

    <<  "\t\t--parallelization <integer> : Specify the parallelization "
    <<  "strategy\n"
    <<  "\t\t\t0 for a static/dynamic chunk based parallelization and 1 "
    <<  "for a\n\t\t\twarehouse based one."
    <<  "\n\t\t\t\tBy default: Static/dynamic chunk based strategy is used"
    <<  "\n\n"

    <<  "\t\t-j or --njobs or --nthreads <integer> : Specify the number of"
    <<  "\n\t\t\tjobs to be used to compute the simulation\n"
    <<  "\t\t\t\tBy default: all available threads are used\n\n"

    <<  "\t\t--chunkSize <integer> : Specify the chunk size to be used for"
    <<  "\n\t\t\tparallel computing. If a negative number is given, then "
    <<  "its"
    <<  "\n\t\t\tabsolute value is used as starting size of the dynamic "
    <<  "chunk-size strategy."
    <<  "\n\t\t\tPositive numbers specify the size for a static chunk-size "
    <<  "strategy"
    <<  "\n\t\t\t\tBy default: 32\n\n"

    <<  "\t\t--warehouseFactor <integer> : Specify the warehouse factor."
    <<  "\n\t\t\tThe number of tasks in the warehouse would be k times the "
    <<  "\n\t\t\tnumber of workers. The greater the factor, the less the "
    <<  "\n\t\t\tprobability of idle cores but the greater the memory "
    <<  "consumption."
    <<  "\n\t\t\t\tBy default: 4\n\n"

    <<  "\t\t--rebuildScene : Force scene rebuild even when a previously\n"
    <<  "\t\t\tbuilt scene is available\n"
    <<  "\t\t\t\tBy default: previous scene is used if found\n\n"

    << "\t\t--noSceneWriting : If a scene is created during asset loading,\n"
    << "\t\t\tit will be written by default. Enabling this flag will prevent\n"
    << "\t\t\tthis writing.\n\n"


    <<  "\t\t--kdt <integer> : Specify the type of KDTree to be built for "
    <<  "for the scene\n"
    <<  "\t\t\tUsing 1 is for the simple KDTree based on median "
    <<  " balancing, 2 for \n"
    <<  "\t\t\tthe SAH based KDTree, 3 for the SAH with best axis\n"
    <<  "\t\t\tbased KDTree and 4 (default) for a fast SAH "
    <<  "approximation\n\n"

    <<  "\t\t--kdtJobs <integer> : Specify the number of threads to be used"
        " for building the KDTree.\n"
        "\t\t\tIf 1, then the KDTree will be built in a sequential fashion"
        "\n\t\t\tIf >1, then the KDTree will be built in a parallel fashion"
        "\n\t\t\tIf 0, then the KDTree will be built using as many threads "
        "as available\n\n"

    <<  "\t\t--kdtGeomJobs <integer> : Specify the number of threads to "
        "be used for building the\n\t\t\t"
        "upper nodes of the KDTree (geometry-level parallelization).\n"
        "\t\t\tIf 1, then there is no geometry-level parallelization"
        "\n\t\t\tIf >1, then geometry-level parallelization uses as many "
        "threads as specified."
        "\n\t\t\tIf 0 (default), then geometry-level parallelization uses"
        "\n\t\t\tas many threads as node-level.\n\n"


    <<  "\t\t--sahNodes <integer> : Specify how many nodes must be used by "
    <<  "the\n"
    <<  "\t\t\tSurface Area Heuristic when building a SAH based KDTree\n"
    <<  "\t\t\tFor the SAH KDTree it is recommended to be 21.\n"
    <<  "\t\t\tMore nodes lead to a best search process\n"
    <<  "\t\t\tto find split position, at the expenses of a\n"
    <<  "\t\t\tgreater computational cost. When using a fast SAH\n"
    <<  "\t\t\tapproximation it is recommended to set this to 32 (default)"
    <<  ".\n\n"

    <<  "\t\t--disablePlatformNoise : Disable platform noise, no matter\n"
    <<  "\t\t\twhat is specified on XML files\n"
    <<  "\t\t\t\tBy default: XML specifications are considered\n\n"

    <<  "\t\t--disableLegNoise : Disable leg noise, no matter what is\n"
    <<  "\t\t\tspecified on XML files\n"
    <<  "\t\t\t\tBy default: XML specifications are considered\n\n"

    <<  "\t\t--logFile : Logging will be outputted to a file, not only\n"
    <<  "\t\t\tto standard output.\n"
    <<  "\t\t\t\tBy default: logging will be written to standard output\n\n"

    <<  "\t\t--logFileOnly : Logging will be outputted ONLY to a file\n"
    <<  "\t\t\tBy default: logging will be outputted to standard output\n\n"

    <<  "\t\t--silent : Disable logging output\n"
    <<  "\t\t\tBy default: only information and errors are reported\n\n"

    <<  "\t\t-q or --quiet : Specify the verbosity level to errors only\n"
    <<  "\t\t\tBy default: only information and errors are reported\n\n"

    <<  "\t\t-vt : Specify the verbosity level to time and errors only\n"
    <<  "\t\t\tBy default: only information and errors are reported\n\n"


    <<  "\t\t-v : Specify the verbosity level to errors, information and "
    <<  "warnings\n"
    <<  "\t\t\tBy default: only information and errors are reported\n\n"

    <<  "\t\t-v2 or -vv : Specify the verbosity level to report all "
    <<  "messages\n"
    <<  "\t\t\tBy default: only information and errors are reported\n\n"

    #ifdef PCL_BINDING
    <<  "\n\n\tDEV-MODE ONLY ARGUMENTS:\n\n"
        "\t\t--demo <demo_name>\n"
        "\t\t\tRun demo with given name.\n"
        "\t\t\t\tFor example: --demo simple_primitives\n\n"
    #endif
    <<  std::endl;
}

}}