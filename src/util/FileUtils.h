#pragma once

#include <string>
#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class with util functions to work with files
 */
class FileUtils{
public:
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
};

