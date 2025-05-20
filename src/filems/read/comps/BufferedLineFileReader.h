#pragma once

#include <filems/read/comps/LineFileReader.h>
#include <filems/read/strategies/BufferedReadingStrategy.h>
#include <filems/read/strategies/LineReadingStrategy.h>
#include <optional>
namespace helios { namespace filems{

class BufferedLineFileReader : public LineFileReader{
protected:
    // ***  USING  *** //
    // *************** //
    using LineFileReader::readingStrategy;
    using LineFileReader::maxCharsPerLine;
    using LineFileReader::ifs;

    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The line reading strategy to be used for each single reading
     *  operation
     * @see filems::LineReadingStrategy
     */
    std::optional<LineReadingStrategy> lrs;
    /**
     * @brief The buffer size for the buffered strategy
     */
    size_t bufferSize;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for buffered line file reader
     */
    BufferedLineFileReader(
        string const &path,
        std::ios_base::openmode openMode = std::ios_base::in,
        long const maxCharsPerLine = 8192,
        size_t const bufferSize = 100000
    ) :
        LineFileReader(path, openMode, maxCharsPerLine, false),
        lrs(std::in_place, ifs, this->maxCharsPerLine),
        bufferSize(bufferSize)
    {makeBufferedStrategy();}
    ~BufferedLineFileReader() override = default;

protected:
    // ***  INNER METHODS  *** //
    // *********************** //
    /**
     * @brief Build a buffered line reading strategy for the buffered line
     *  file reader
     * @see LineFileReader::makeStrategy
     */
    void makeStrategy() override{
        lrs.emplace(ifs, maxCharsPerLine);
        makeBufferedStrategy();
    }
    /**
     * @brief Define the building of the buffered reading strategy itself
     * @see BufferedLineFileReader::makeStrategy
     */
    virtual void makeBufferedStrategy() {
        readingStrategy = make_shared<BufferedReadingStrategy<string>>(
            *lrs,
            bufferSize
        );
    }

public:
    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain the buffer size for the buffered line file reader
     * @return The buffer size of the buffered line file reader
     * @see BufferedLineFileReader::bufferSize
     * @see BufferedLineFileReader::setBufferSize
     */
    virtual inline size_t getBufferSize() {return bufferSize;}
    /**
     * @brief Get the buffer size for the buffered line file reader
     * @param bufferSize The new buffer size for the buffered line file reader
     * @see BufferedLineFileReader::bufferSize
     * @see BufferedLineFileReader::getBufferSize
     */
    virtual void setBufferSize(size_t const bufferSize){
        this->bufferSize = bufferSize;
        makeBufferedStrategy();
    }
};

}}