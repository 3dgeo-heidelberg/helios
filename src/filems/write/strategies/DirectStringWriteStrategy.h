#pragma once

#include <filems/write/strategies/WriteStrategy.h>

#include <string>
#include <fstream>
#include <sstream>

namespace helios { namespace filems{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing WriteStrategy to directly write
 *  strings to a file
 * @see filems::WriteStrategy
 * @see filems::SimpleSyncFileStringWriter
 */
class DirectStringWriteStrategy : public WriteStrategy<std::string const &> {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The output file stream to do the writing
     */
    std::ofstream &ofs;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for direct string write strategy
     * @see DirectStringWriteStrategy::ofs
     */
    DirectStringWriteStrategy(std::ofstream &ofs) : ofs(ofs) {}
    virtual ~DirectStringWriteStrategy() {}

    // ***  WRITE STRATEGY INTERFACE *** //
    // ********************************* //
    /**
     * @brief Write string to file
     * @param str String to be written
     */
    void write(std::string const & str) override{
        ofs << str;
    }
};

}}