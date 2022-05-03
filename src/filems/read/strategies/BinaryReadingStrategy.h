#pragma once

#include <filems/read/strategies/SimpleReadingStrategy.h>

#include <fstream>

namespace helios { namespace filems{

using std::fstream;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class defining the strategy to read byte by byte from a file input
 *  stream
 * @see filems::SimpleReadingStrategy
 */
class BinaryReadingStrategy : public SimpleReadingStrategy<int> {
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for binary reading strategy
     * @see filems::SimpleReadingStrategy::SimpleReadingStrategy
     */
    BinaryReadingStrategy(ifstream &ifs) :
        SimpleReadingStrategy<int>(ifs)
    {}
    virtual ~BinaryReadingStrategy() = default;

    // ***  READING STRATEGY METHODS  *** //
    // ********************************** //
    /**
     * @brief Read from input file stream
     * @return Either what has been read from input file or EOF if end of file
     *  was reached
     */
    int read() override{
        return ifs.get();
    };

};

}}