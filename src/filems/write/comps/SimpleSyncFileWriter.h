#pragma once

#include <filems/write/comps/SyncFileWriter.h>
#include <MathConverter.h>

#include <fstream>
#include <string>
#include <iomanip>
#include <ostream>
#include <iterator>

namespace helios { namespace filems{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract specialization of SyncFileWriter to write output directly
 *  to a file
 * @see filems::SyncFileWriter
 */
template <typename ... WriteArgs>
class SimpleSyncFileWriter : public SyncFileWriter<WriteArgs ...>{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Output file stream to be used by the simple synchronous file
     *  writer
     */
	std::ofstream ofs;
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Simple synchronous file writer constructor
     * @param path Path to the output file
     * @param om Open mode for the file (append by default)
     */
	explicit SimpleSyncFileWriter(
	    const std::string& path,
	    std::ios_base::openmode om = std::ios_base::app
    ) :
	    SyncFileWriter<WriteArgs ...>(path)
    {
		// Open file for writing ...
		ofs.open(path, om);
		ofs.exceptions(
		    std::ios_base::eofbit |
		    std::ios_base::failbit |
		    std::ios_base::badbit
        );
	}
	virtual ~SimpleSyncFileWriter() {finish();}

    // ***  F I N I S H  *** //
    // ********************* //
    /**
     * @brief SimpleSyncFileWriter finish method does not do nothing. The
     * writing operations are guaranteed to be done after the instance has
     * been destroyed.
     */
    void finish() override {if(ofs.is_open()) ofs.close();}
};

}}
