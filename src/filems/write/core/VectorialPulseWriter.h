#pragma once

#include <util/HeliosException.h>
#include <filems/write/core/BasePulseWriter.h>
#include <filems/write/comps/SimpleVectorialSyncFilePulseWriter.h>
#include <filems/write/comps/ZipVectorialSyncFilePulseWriter.h>
#include <scanner/PulseRecord.h>

#include <glm/glm.hpp>

#include <string>
#include <memory>

namespace helios { namespace filems{

using std::string;
using std::shared_ptr;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Common implementation for any vectorial pulse writer
 */
class VectorialPulseWriter :
    public BasePulseWriter<vector<PulseRecord> const &>
{
protected:
    // ***  USING  *** //
    // *************** //
    using BasePulseWriter<vector<PulseRecord> const &>::writers;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for vectorial pulse writer
     */
    VectorialPulseWriter() :
            BasePulseWriter<vector<PulseRecord> const &>()
    {}
    virtual ~VectorialPulseWriter() = default;

    // ***   M E T H O D S   *** //
    // ************************* //
    /**
     * @brief Writer a vector of pulses (represented by many vectors of the
     *  same dimensionality with one component per pulse).
     */
    void writePulses(vector<PulseRecord> const & pulseRecords);
    /**
     * @brief Like filems::VectorialPulseWriter::writePulses but faster because
     *  there is no validation
     * @see filems::VectorialPulseWriter::writePulses
     */
    inline void writePulsesUnsafe(vector<PulseRecord> const & pulseRecords){
        sfw->write(pulseRecords);
    }
    /**
     * @brief Make a vectorial pulse SyncFileWriter
     * @see BasePulseWriter::makeWriter
     */
    shared_ptr<SyncFileWriter<std::vector<PulseRecord> const &>> makeWriter(
        string const &path
    ) const override {
        if(isZipOutput()){
            return make_shared<ZipVectorialSyncFilePulseWriter>(path);
        }
        else{
            return make_shared<SimpleVectorialSyncFilePulseWriter>(path);
        }
    }

};

}}