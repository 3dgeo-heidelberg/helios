#pragma once

#include <string>
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
     * @return true if help was requested, false otherwise.
     */
    bool parseHelpRequest();
    /**
     * @brief Parse test request argument if any (--test)
     * @return true if test was requested, false otherwise.
     */
    bool parseTestRequest();
    /**
     * @brief Parse the survery path from invocation arguments
     * @return Parsed survey path
     */
    std::string parseSurveyPath();
    /**
     * @brief Parse the assets path from invocation arguments
     * @return Parsed assets path or default value if none was found
     */
    std::string parseAssetsPath();
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
     * @brief Parse the number of jobs from invocation arguments
     * @return Parsed number of jobs. If no number of jobs was specified,
     * 0 will be returned.
     */
    std::size_t parseNJobs();
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
    int findIndexOfArgument(std::string&& arg);
};
