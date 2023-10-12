#pragma once

#include <filems/write/strategies/WriteStrategy.h>
#include <scanner/PulseRecord.h>

#include <glm/glm.hpp>

#include <fstream>
#include <sstream>
#include <iomanip>

namespace helios { namespace filems{


/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing WriteStrategy to directly write pulse
 *  data to a file.
 * @see filems::WriteStrategy
 * @see filems::SimpleSyncFilePulseWriter
 */
class DirectPulseWriteStrategy : public WriteStrategy<PulseRecord const &>{
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
     * @brief Default constructor for direct pulse write strategy
     * @see DirectPulseWriteStrategy::ofs
     */
    DirectPulseWriteStrategy(std::ofstream &ofs) : ofs(ofs) {}
    virtual ~DirectPulseWriteStrategy() = default;

    // ***  WRITE STRATEGY INTERFACE  *** //
    // ********************************** //
    /**
     * @brief Write pulse data to file
     * @see SyncFileWriter::_write
     */
    void write(PulseRecord const & pulseRecord) override {
        ofs << pulseToString(pulseRecord);
    }

protected:
    // ***  UTILS  *** //
    // *************** //
    /**
     * @brief Build a string from pulse data
     * @return String with pulse data
     */
    virtual std::string pulseToString(PulseRecord const &pulseRecord) {
        std::stringstream ss;
        ss  << std::setprecision(4) << std::fixed
            << pulseRecord.ox << " "
            << pulseRecord.oy << " "
            << pulseRecord.oz << " "
            << pulseRecord.vx << " "
            << pulseRecord.vy << " "
            << pulseRecord.vz << " "
            << pulseRecord.time << " "
            << pulseRecord.pulseIndex << " "
            << pulseRecord.deviceIndex << "\n";
        return ss.str();
    }

};

}}
