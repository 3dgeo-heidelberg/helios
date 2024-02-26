#pragma once

#include <string>
#include <vector>
#include <map>

#include <util/typedef.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class with util functions to work with files
 */
class FileUtils{
public:
    // ***  CONSTANTS  *** //
    // ******************* //
    /**
     * @brief Path separator constant
     */
    static char const pathSeparator;

    // ***  METHODS  *** //
    // ***************** //
    /**
     * @brief Handle the filepath or efilepath argument from a map of
     * parameters in the context of geometry loading
     * @param params Map of parameters defining the geometry loading process
     * @return Vector of parsed file paths (1 for filepath, n for efilepath)
     */
    static std::vector<std::string> handleFilePath(
        std::map<std::string, ObjectT> & params,
        const std::vector<std::string> & assetsDir
    );

    /**
     * @brief Obtain the files which are referenced by the path expression
     *
     * For instance, the expression "/tmp/heat.*data.csv" would obtain all
     * files in /tmp/ directory which start by "heat" and end by "data.csv"
     *
     * Another example, using expression "/home/helios/sceneparts/.*\.obj"
     * would obtain all files in "/home/helios/sceneparts/" which end by
     * ".obj"
     *
     * @param pathExpression The path expression itself.
     * For instance "/tmp/.*\.csv"
     */
    static std::vector<std::string> getFilesByExpression(
        std::string const pathExpression
    );

    /**
     * @brief Decompress a file generated through ZipSyncFileWriter
     * @param inputPath Path to the compressed file to be decompressed
     * @param outputPath Path where the decompressed file will be written
     * @see ZipSyncFileWriter
     */
    static void unzipFile(
        std::string const inputPath,
        std::string const outputPath
    );

    /**
     * @brief Extract the extension and the path without extension from given
     *  path with extension
     * @param[in] path The patch which extension must be extracted
     * @param[out] ext Where the extracted extension will be stored
     * @param[out] pathNonExt Where the path without extension will be stored
     */
    static void extractExtensionAndPathWithoutExtension(
        std::string const &path,
        std::string &ext,
        std::string &pathNonExt
    );
    /**
     * @brief Build a string such that: builtString = pathNonExt + suffix + ext
     * @param pathNonExt The path without extension
     * @param suffix The suffix to be appended to the path without extension
     * @param ext The extension to be appended after the suffix
     * @return Built string : pathNonExt + suffix + ext
     */
    static std::string craftPathWithSuffix(
        std::string const &pathNonExt,
        std::string const &suffix,
        std::string const &ext
    );
};

