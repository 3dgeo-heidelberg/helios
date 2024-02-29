#pragma once

#include <string>
#include <vector>
#include <logging.hpp>

/**
 * @author Alberto M. Esmoris Pena
 * @verison 1.0
 * @brief Parser for helios-plusplus invocation arguments
 */
class ArgumentsParser {
public:
    // *** CONSTRUCTOR *** //
    // ******************* //
    /**
     * @brief Instantiate an ArgumentsParser
     * @param argc Number of arguments
     * @param argv Array of arguments
     */
    ArgumentsParser(int argc, char **argv) : argc(argc), argv(argv) {}

    // *** PUBLIC METHODS *** //
    // ********************** //
    /**
     * @brief Parse help request argument if any (-h or --help)
     * @return True if help was requested, false otherwise.
     */
    bool parseHelpRequest();
    /**
     * @brief Parse test request argument if any (--test)
     * @return True if test was requested, false otherwise.
     */
    bool parseTestRequest();
    /**
     * @brief Parse version request argument if any (--version)
     * @return True if version was requested, false otherwise.
     */
    bool parseVersionRequest();
    /**
     * @brief Parse test directory specification (not necessary,
     *  default one will be relative path "data/test/")
     * @return Parsed test directory path
     */
    std::string parseTestDir();
    /**
     * @brief Parse demo request argument if any (--demo)
     * @return Parsed demo if any or "NULL" if no demo was requested
     */
    std::string parseDemoRequest();
    /**
     * @brief Parse the demo survey path from invocation arguments
     * @return Parsed demo survey path if any, empty string "" otherwise
     */
    std::string parseDemoSurveyPath();
    /**
     * @brief Parse the demo assets path from invocation arguments
     * @return Parsed demo assets path if any, empty string "" otherwise
     */
    std::string parseDemoAssetsPath();
    /**
     * @brief Parse the survey path from invocation arguments
     * @return Parsed survey path
     */
    std::string parseSurveyPath();
    /**
     * @brief Parse the assets paths from invocation arguments
     * @return Parsed assets path or default value if none was found
     */
    std::vector<std::string> parseAssetsPath();
    /**
     * @brief Parse the output path from invocation arguments
     * @return Parsed output path or default value if none was found
     */
    std::string parseOutputPath();
    /**
     * @brief Parse the write waveform flag specification
     * @return True if write waveform flag was enabled, False otherwise
     */
    bool parseWriteWaveform();
    /**
     * @brief Parse the write pulse flag specification
     * @return True if write pulse flag was enabled, False otherwise
     */
    bool parseWritePulse();
    /**
     * @brief Parse the calc echo width flag specification
     * @return True if calc echo width flag was enabled, False otherwise
     */
    bool parseCalcEchowidth();
    /**
     * @brief Parse the seed for RandomnessGenerator from invocation arguments
     * @return Seed for RandomnessGenerator
     */
    std::string parseSeed();
    /**
     * @brief Parse the parallelization strategy from invocation arguments
     * @return 0 for chunk based parallelization, 1 for warehouse based
     *  parallelization (default)
     */
    int parseParallelizationStrategy();
    /**
     * @brief Parse the number of jobs from invocation arguments
     * @return Parsed number of jobs. If no number of jobs was specified,
     * 0 will be returned.
     */
    std::size_t parseNJobs();
    /**
     * @brief Parse the chunk size for the pulse task dropper from invocation
     *  arguments
     * @return Parsed chunk size. If no chunk size was specified, 32 will
     *  be returned as default value
     */
    int parseChunkSize();
    /**
     * @brief Parse the warehouse factor for the warehouse based
     *  parallelization strategy from invocation arguments
     * @return Parsed warehouse factor. If no warehouse factor was specified,
     *  4 will be returned as default value
     */
    int parseWarehouseFactor();
    /**
     * @brief Parse the disable platform noise flag from invocation arguments
     * @return True if disable platform noise flag was specified,
     * False otherwise
     */
    bool parseDisablePlatformNoise();
    /**
     * @brief Parse the disable leg noise flag from invocation arguments
     * @return True if disable leg noise flag was specified,
     * False otherwise
     */
    bool parseDisableLegNoise();
    /**
     * @brief Parse the rebuild scene flag from invocation arguments
     * @return True if rebuild scene flag was specified, False otherwise
     */
    bool parseRebuildScene();
    /**
     * @brief Parse the verbosity level for logging from invocation arguments
     */
    void parseLoggingVerbosity();
    /**
     * @brief Parse the logging output mode from invocation arguments
     * @return Translated parsed logging output mode. By default "std_out"
     */
    std::string parseLoggingOutputMode();
    /**
     * @brief Parse the full wave noise specification.
     * @return True if full wave noise was setted, False otherwise.
     */
    bool parseFullWaveNoise();
    /**
     * @brief Parse the LAS output specification
     * @return True if LAS output was requested, False otherwise.
     */
    bool parseLasOutput();
    /**
     * @brief Parse the LAS version output specification
     * @return True if LAS v1.0 was requested, False otherwise.
     */
    bool parseLas10();
    /**
     * @brief Parse the ZIP output specification
     * @return True if ZIP output was requested, False otherwise.
     */
    bool parseZipOutput();
    /**
     * @brief Parse the scale factor to be used by the LasSyncFileWriter
     * @return Scale factor to be used by the LasSyncFileWriter
     */
    double parseLasScale();
    /**
     * @brief Parse an unzip request for given input and output path
     * @param inputPath Where the input path will be stored
     * @param outputPath Where the output path will be stored
     * @return True if unzip was requested, False otherwise.
     */
    bool parseUnzip(std::string *inputPath, std::string *outputPath);
    /**
     * @brief Parse the fixed incidence angle specification.
     * When fixed incidence angle is requested, incidence angle for all
     * primitives will be exactly 1.0
     * @return True if fixed incidence angle was requested, False otherwise.
     */
    bool parseFixedIncidenceAngle();
    /**
     * @brief Parse the fixed GPS time start for Simulation
     * @return Parsed fixed GPS time start. By default it is an empty string,
     *  which leads to use current local time.
     * @see Simulation::fixedGpsTimeStart
     * @see Simulation::Simulation
     * @see Simulation
     */
    std::string parseGpsStartTime();
    /**
     * @brief Parse the type of KDTree
     *
     * 1 : The simple KDTree built based on balancing through median
     *  heuristic
     *
     * 2 : The SAH KDTree built based on surface area heuristic
     *
     * 3 : The SAH KDTree built based on surface area heuristic and best axis
     *
     * 4 (default) : The Fast SAH KDtree built based on a fast iterative
     *  approximation of SAH
     *
     * @return Number identifying the type of KDTree to be built if necessary
     */
    int parseKDTreeType();
    /**
     * @brief Parse how many KDTree jobs must be used to build the KDTree
     *
     * 1 : Sequential building
     *
     * 0 (default) : As many threads as available by the system
     *
     * >1 : Exactly this number of threads for parallel building
     *
     * @return Number of jobs to be used to build the KDTree
     */
    size_t parseKDTreeJobs();
    /**
     * @brief Parse how many KDTree geometry-level jobs must be used to build
     *  the KDTree upper nodes
     *
     * 1 : Only node-level parallelization, which corresponds with only 1
     *  node at geometry-level parallelization
     *
     * 0 (default) : As many threads as KDTree jobs
     *
     * >1 : Exactly this number of threads for geometry-level parallel building
     *
     * @return Number of jobs to be used at geometry-level parallelization
     *  of KDTree building
     */
    size_t parseKDTreeGeometricJobs();
    /**
     * @brief Parse on how many nodes the loss function of the surface area
     *  heuristic must be evaluated when building the KDTree.
     *  For the Fast SAH it is the number of iterations computed to approximate
     *  SAH
     *
     * @return Number of nodes to evaluate loss function when building KDTree
     *  with a surface area heuristic approach. Number of iterations to
     *  approximate SAH when using Fast SAH strategy
     */
    size_t parseSAHLossNodes();
    /**
     * @brief Parse whether the output point clouds must be exported on a
     *  different file per channel or not
     * @return True if the output point clouds must be exported on a different
     *  file par channel, False if the output point clouds must be exported
     *  on the same file with no concern for the channel
     */
    bool parseSplitByChannel();
    /**
     * @brief Parse whether the legacy energy model must be used (true) or not
     *  (false).
     * @return True if the legacy energy model must be used, false otherwise.
     * @see EnergyModel
     * @see BaseEnergyModel
     * @see ImprovedEnervyModel
     * @see ScanningDevice
     */
    bool parseLegacyEnergyModel();


private:
    // *** PRIVATE ATTRIBUTES *** //
    // ************************** //
    /**
     * @brief Number of arguments in array or arguments (argv)
     * @see ArgumentsParser::argv
     */
    int argc;
    /**
     * @brief Array of arguments
     * @see ArgumentsParser::argc
     */
    char **argv;

    // *** PRIVATE METHODS *** //
    // *********************** //
    /**
     * @brief Find the index of received argument
     * @param arg Argument to be found
     * @return Index Index of the argument to be found, -1 if it was not found
     */
    int findIndexOfArgument(std::string&& arg, int offset = 0);
};
