#pragma once

#include <filems/write/core/HeliosWriter.h>
#include <filems/write/comps/SimpleSyncFileWriter.h>
#include <filems/write/comps/ZipSyncFileWriter.h>

#include <boost/filesystem.hpp>
#include <glm/glm.hpp>

#include <string>
#include <memory>
#include <vector>

namespace fs=boost::filesystem;

namespace helios { namespace filems{

using std::string;
using std::shared_ptr;
using std::vector;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class to handle writing of full waveform to generate HELIOS++ output
 *  virtual full waveform
 */
class FullWaveformWriter : public HeliosWriter<
    std::vector<double> const &,
    int const,
    double const,
    double const,
    glm::dvec3 const&,
    glm::dvec3 const&,
    long const
>{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for full waveform writer
     */
    FullWaveformWriter() = default;
    virtual ~FullWaveformWriter() = default;

    // ***   M E T H O D S   *** //
    // ************************* //
    /**
     * @brief Configure the output path for the full waveform writer
     * @param parent Path to output directory for full waveform files
     * @param prefix Prefix for the name of the output file
     * @param computeWaveform Flag to specify if waveform must be computed
     *  (true) or not (false)
     */
    void configure(
        string const &parent,
        string const &prefix,
        bool const computeWaveform
    );
    /**
     * @brief Write full waveform data
     */
    void writeFullWaveform(
        vector<double> const &fullwave,
        int const fullwaveIndex,
        double const minTime,
        double const maxTime,
        glm::dvec3 const &beamOrigin,
        glm::dvec3 const &beamDir,
        double const gpsTime
    );
    /**
     * @brief Like filems::FullWaveformWriter::writeFullWaveform but faster
     *  because there is no validation
     * @see filems::FullWaveformWriter::writeFullWaveform
     */
    inline void writeFullWaveformUnsafe(
        vector<double> const &fullwave,
        int const fullwaveIndex,
        double const minTime,
        double const maxTime,
        glm::dvec3 const &beamOrigin,
        glm::dvec3 const &beamDir,
        double const gpsTime
    ) const
    {
        sfw->write(
            fullwave,
            fullwaveIndex,
            minTime,
            maxTime,
            beamOrigin,
            beamDir,
            gpsTime
        );
    }

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Set the path to output file
     * @param path New path to output file
     */
    void setOutputFilePath(string const &path);
    /**
     * @brief Get the path to the output file
     * @return The path to the output file
     * @see filems::FullWaveformWriter::getOutputPath
     */
    fs::path getOutputFilePath() const {return fs::path(getOutputPath());}
    /**
     * @see filems::FullWaveformWriter::getOutputFilePath
     */
    string getOutputPath() const {return sfw->getPath();}
};

}}