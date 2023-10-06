#pragma once

#include <filems/facade/FMSWriteFacade.h>
#include <util/WriteYielder.h>
#include <scanner/PulseRecord.h>

#include <cstdlib>
#include <mutex>
#include <vector>

using helios::filems::FMSWriteFacade;

using std::size_t;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Yield a set of pulse record from single pulse data so it is written
 *  when buffer size has been reached or, alternatively, when yielder is
 *  directly forced to yield.
 */
class PulseRecordYielder : public WriteYielder<PulseRecord>{
protected:
    // ***  USING  *** //
    // *************** //
    using WriteYielder<PulseRecord>::write;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for pulse record yielder
     * @see WriteYielder::write
     * @see Yielder::bufferSize
     */
    PulseRecordYielder(
        FMSWriteFacade &write,
        size_t bufferSize=256
    ) :
        WriteYielder<PulseRecord>(write, bufferSize)
    {}
    virtual ~PulseRecordYielder() = default;

    // ***  YIELD METHODS  *** //
    // *********************** //
    /**
     * @brief Write the temporal copy of the pulse record through the write
     *  facade of the filems
     * @param copy The temporal copy of pulse records buffer to be digested
     * @see WriteYielder
     */
    void digest(vector<PulseRecord> &copy) override {
        write.writePulsesUnsafe(copy);
    }

};