#pragma once

#include <filems/write/strategies/WriteStrategy.h>

#include <vector>
#include <fstream>
#include <sstream>

namespace helios { namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing WriteStrategy to directly write full
 *  waveform data to a file.
 * @see filems::WriteStrategy
 * @see filems::SimpleSyncFileFullWaveformWriter
 */
class DirectFullWaveformWriteStrategy :
    public WriteStrategy<
        std::vector<double> const&,
        int const,
        double const,
        double const,
        glm::dvec3 const&,
        glm::dvec3 const&,
        long const
    >
{
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
     * @brief Default constructor for direct full waveform write strategy
     * @see DirectFullWaveformWriteStrategy::ofs
     */
    DirectFullWaveformWriteStrategy(std::ofstream &ofs) : ofs(ofs) {}
    virtual ~DirectFullWaveformWriteStrategy() = default;

    // ***  WRITE STRATEGY INTERFACE  *** //
    // ********************************** //
    /**
     * @brief Write full waveform data to file
     * @param fullwave Fullwave data
     * @param fullwaveIndex Fullwave index
     * @param minTime Min hit time for the fullwave (ns)
     * @param maxTime Max hit time for the fullwave (ns)
     * @param beamOrigin Origin for the beam
     * @param beamDir Director vector for the beam
     * @param gpsTime GPS time
     * @see SyncFileWriter::_write
     */
    void write(
        std::vector<double> const &fullwave,
        int const fullwaveIndex,
        double const minTime,
        double const maxTime,
        glm::dvec3 const & beamOrigin,
        glm::dvec3 const & beamDir,
        long const gpsTime
    ) override {
        ofs << fullwaveToString(
            fullwave,
            fullwaveIndex,
            minTime,
            maxTime,
            beamOrigin,
            beamDir,
            gpsTime
        );
    }

protected:
    // ***  UTILS  *** //
    // *************** //
    /**
     * @brief Build a string from fullwave data
     * @param fullwave Fullwave data
     * @param fullwaveIndex Fullwave index
     * @param minTime Min hit time for the fullwave (ns)
     * @param maxTime Max hit time for the fullwave (ns)
     * @param beamOrigin Origin for the beam
     * @param beamDir Director vector for the beam
     * @param gpsTime GPS time
     * @return String with fullwave data
     */
    virtual std::string fullwaveToString(
        std::vector<double> const &fullwave,
        int const fullwaveIndex,
        double const minTime,
        double const maxTime,
        glm::dvec3 const & beamOrigin,
        glm::dvec3 const & beamDir,
        long const gpsTime
    ){
        std::stringstream ss;
        ss << std::setprecision(4) << std::fixed;
        ss << fullwaveIndex << " "
           << beamOrigin.x << " "
           << beamOrigin.y << " "
           << beamOrigin.z << " "
           << beamDir.x << " "
           << beamDir.y << " "
           << beamDir.z << " "
           << minTime << " "
           << maxTime << " "
           << ((double)gpsTime)/1000.0 << " ";

        std::copy(
            fullwave.begin(),
            fullwave.end()-1,
            std::ostream_iterator<double>(ss, " ")
        );
        ss << fullwave.back();
        ss << "\n";
        return ss.str();
    }
};

}}