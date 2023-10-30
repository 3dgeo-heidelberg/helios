#pragma once

#include <util/HeliosException.h>
#include <filems/write/core/BasePulseWriter.h>
#include <filems/factory/SyncFileMeasurementWriterFactory.h>
#include <filems/write/comps/SimpleSyncFilePulseWriter.h>
#include <filems/write/comps/ZipSyncFilePulseWriter.h>
#include <scanner/PulseRecord.h>

namespace helios { namespace filems{

using std::string;
using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class to handle writing of pulses as HELIOS++ output
 */
class PulseWriter : public BasePulseWriter<PulseRecord const &>{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for pulse writer
     */
    PulseWriter() = default;
    virtual ~PulseWriter() = default;

    // ***   M E T H O D S   *** //
    // ************************* //
    /**
     * @brief Write pulse data
     */
    void writePulse(PulseRecord const &pulseRecord);
    /**
     * @brief Like filems::PulseWriter::writePulse but faster because there is
     *  no validation
     * @see filems::PulseWriter::writePulse
     */
    inline void writePulseUnsafe(PulseRecord const &pulseRecord)
    {sfw->write(pulseRecord);}
    /**
     * @brief Make a single pulse SyncFileWriter
     * @see BasePulseWriter::makeWriter
     */
    shared_ptr<SyncFileWriter<PulseRecord const &>> makeWriter (
        string const &path
    ) const override{
        if(isZipOutput()){
            return make_shared<ZipSyncFilePulseWriter>(path);
        }
        else return make_shared<SimpleSyncFilePulseWriter>(path);
    }
};

}}