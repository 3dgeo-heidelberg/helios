#pragma once

#include <filems/read/comps/FileReader.h>

#include <fstream>
#include <memory>

namespace helios { namespace filems{

using std::string;
using std::ifstream;
using std::ios_base;
using std::make_shared;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract class defining the fundamental of any file reader that uses
 *  standard file input stream as reading mechanism
 * @tparam ReadArg Type of what is read from file
 * @see filems::FileReader
 */
template <typename ReadArg>
class SimpleFileReader : public FileReader<ReadArg>{
protected:
    // ***  USING  *** //
    // *************** //
    using FileReader<ReadArg>::readingStrategy;
    using FileReader<ReadArg>::makeStrategy;

    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The input file stream to read from
     */
    ifstream ifs;
    /**
     * @brief The open mode flags for the input file stream
     * @see filems::SimpleFileReader::ifs
     */
    ios_base::openmode openMode;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for simple file reader
     * @see filems::FileReader::FileReader
     * @see filems::SimpleFileReader::ifs
     * @see filems::SimpleFileReader::openMode
     */
    SimpleFileReader(
        string const &path,
        ios_base::openmode openMode = ios_base::in
    ) :
        FileReader<ReadArg>(path),
        ifs(path, openMode),
        openMode(openMode)
    {}
    virtual ~SimpleFileReader() = default;

    // ***  READ METHODS  *** //
    // ********************** //
    /**
     * @brief Read from file simply by applying the reading strategy.
     *  Therefore, there is no concurrency handling mechanism and usage of
     *  simple file reader is not thread safe
     * @see filems::FileReader::read
     * @see filems::SimpleReadingStrategy
     */
    ReadArg read() override {return readingStrategy->read();};

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Set the path to the file to be read, also opening the input
     *  stream for the new file and updating the strategy. It assures the
     *  previous input file stream is closed before opening the new one.
     * @see filems::FileReader::setPath
     */
    void setPath(string const &path) override {
        FileReader<string>::setPath(path);
        ifs.close();
        ifs = ifstream(path, openMode);
        makeStrategy();
    }
};

}}