#pragma once

#include <filems/write/core/MeasurementWriter.h>
#include <filems/write/core/TrajectoryWriter.h>
#include <filems/write/core/FullWaveformWriter.h>

#include <glm/glm.hpp>

#include <vector>
#include <string>
#include <memory>

namespace helios { namespace filems {

class FMSFacadeFactory;

using std::vector;
using std::string;
using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief The facade for FMS writing
 */
class FMSWriteFacade{
friend class FMSFacadeFactory;
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The writer for measurements
     * @see filems::MeasurementWriter
     */
    shared_ptr<MeasurementWriter> mw = nullptr;
    /**
     * @brief The writer for trajectories
     * @see filems::TrajectoryWriter
     */
    shared_ptr<TrajectoryWriter> tw = nullptr;
    /**
     * @brief The writer for full waveform
     * @see filems::FullWaveformWriter
     */
    shared_ptr<FullWaveformWriter> fww = nullptr;
    /**
     * @brief The root directory for output files
     */
    string rootDir = "./";

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for FMS write facade
     */
    FMSWriteFacade() = default;
    virtual ~FMSWriteFacade() = default;

    // ***  FACADE WRITE METHODS  *** //
    // ****************************** //
    /**
     * @brief Configure the output path for all writers in the facade
     * @param prefix Prefix for the name of the output file
     * @param computeWaveform Flag to specify if waveform must be computed
     *  (true) or not (false)
     * @param lastLegInStrip Specify whether the last leg belonged to a strip
     *  (true) or not (false)
     */
    void configure(
        string const &prefix,
        bool const computeWaveform,
        bool const lastLegInStrip
    );
    /**
     * @brief Obtain the root directory of the write facade
     * @see filems::FMSWriteFacade::rootDir
     */
    inline string getRootDir() const {return rootDir;}
    /**
     * @brief Set the root directory of the write facade
     * @param rootDir The new root directory for the write facade
     * @see filems::FMSWriteFacade:rootDir
     */
    inline void setRootDir(string const &rootDir) {this->rootDir = rootDir;}

    // ***  FACADE MEASUREMENT WRITE METHODS  *** //
    // ****************************************** //
    /**
     * @brief Obtain the measurement writer of the write facade
     * @return The measurement writer of the write facade
     * @see FMSWriteFacade::mw
     */
    inline shared_ptr<MeasurementWriter> getMeasurementWriter() const
    {return this->mw;}
    /**
     * @brief Set the measurement writer of the write facade
     * @param mw New measurement writer for the write facade
     * @see FMSWriteFacade::mw
     */
    inline void setMeasurementWriter(shared_ptr<MeasurementWriter> mw)
    {this->mw = mw;}
    /**
     * @brief Validate the measurement writer of the facade is valid to
     *  support write methods. If it is not valid, an adequate exception
     *  will be thrown.
     * @see FMSWriteFacade::mw
     */
    void validateMeasurementWriter(
        string const &callerName="FMSWriteFacade::validateMeasurementWriter",
        string const &errorMsg="could not access MeasurementWriter"
    ) const;
    /**
     * @see MeasurementWriter::writeMeasurement
     */
    void writeMeasurement(Measurement const &m);
    /**
     * @brief Write the measurement without validations (it is faster than its
     *  non unsafe counterpart)
     * @see MeasurementWriter::writeMeasurement
     */
    inline void writeMeasurementUnsafe(Measurement const &m) const
    {mw->writeMeasurementUnsafe(m);}
    /**
     * @see MeasurementWriter::clearPointcloudFile
     */
    void clearPointcloudFile();
    /**
     * @see filems::HeliosWriter::finish
     */
    void finishMeasurementWriter();
    /**
     * @see filems::MeasurementWriter::getOutputFilePath
     */
    fs::path getMeasurementWriterOutputPath();
    /**
     * @see filems::MeasurementWriter::setOutputFilePath
     */
    void setMeasurementWriterOutputPath(
        std::string path,
        const bool lastLegInStrip
    );
    /**
     * @see filems::MeasurementWriter::isLasOutput
     */
    bool isMeasurementWriterLasOutput() const;
    /**
     * @see filems::MeasurementWriter::setLasOutput
     */
    void setMeasurementWriterLasOutput(bool const lasOutput);
    /**
     * @see filems::MeasurementWriter::isLas10
     */
    bool isMeasurementWriterLas10() const;
    /**
     * @see filems::MeasurementWriter::setLas10
     */
    void setMeasurementWriterLas10(bool const las10);
    /**
     * @see filems::MeasurementWriter::isZipOutput
     */
    bool isMeasurementWriterZipOutput() const;
    /**
     * @see filems::MeasurementWriter::setZipOutput
     */
    void setMeasurementWriterZipOutput(bool const zipOutput);
    /**
     * @see filems::MeasurementWriter::getLasScale
     */
    double getMeasurementWriterLasScale() const;
    /**
     * @see filems::MeasurementWriter::setLasScale
     */
    void setMeasurementWriterLasScale(double const lasScale);


    // ***  FACADE TRAJECTORY WRITE METHODS  *** //
    // ***************************************** //
    /**
     * @brief Obtain the trajectory writer of the write facade
     * @return The trajectory writer of the write facade
     * @see FMSWriteFacade::tw
     */
    inline shared_ptr<TrajectoryWriter> getTrajectoryWriter() const
    {return this->tw;}
    /**
     * @brief Set the trajectory writer of the write facade
     * @param tw New trajectory writer for the write facade
     * @see FMSWriteFacade::tw
     */
    inline void setTrajectoryWriter(shared_ptr<TrajectoryWriter> tw)
    {this->tw = tw;}
    /**
     * @brief Validate the trajectory writer of the facade is valid to support
     *  write methods. If it is not valid, an adequate exception will be
     *  thrown.
     * @see FMSWriteFacade::tw
     */
    void validateTrajectoryWriter(
        string const &callerName="FMSWriteFacade::validateTrajectoryWriter",
        string const &errorMsg="could not accesss TrajectoryWriter"
    ) const;
    /**
     * @see TrajectoryWriter::writeTrajectory
     */
    void writeTrajectory(Trajectory const &t);
    /**
     * @brief Write the trajectory without validations (it is faster than its
     *  non unsafe counterpart)
     * @see TrajectoryWriter::writeTrajectory
     */
    inline void writeTrajectoryUnsafe(Trajectory const &t) const
    {tw->writeTrajectoryUnsafe(t);}
    /**
     * @see filems::HeliosWriter::finish
     */
    void finishTrajectoryWriter();
    /**
     * @see filems::TrajectoryWriter::getOutputFilePath
     */
    fs::path getTrajectoryWriterOutputPath();
    /**
     * @see filems::TrajectoryWriter::setOutputFilePath
     */
    void setTrajectoryWriterOutputPath(string const &path);
    /**
     * @see filems::TrajectoryWriter::isZipOutput
     */
    bool isTrajectoryWriterZipOutput() const;
    /**
     * @see filems::TrajectoryWriter::setZipOutput
     */
    void setTrajectoryWriterZipOutput(bool const zipOutput);


    // ***  FACADE FULL WAVEFORM WRITE METHODS  *** //
    // ******************************************** //
    /**
     * @brief Obtain the full waveform writer of the write facade
     * @return The full waveform writer of the write facade
     * @see FMSWriteFacade::fww
     */
    inline shared_ptr<FullWaveformWriter> getFullWaveformWriter() const
    {return this->fww;}
    /**
     * @brief Set the full waveform writer of the write facade
     * @param fww New full waveform writer for the write facade
     * @see FMSWriteFacade::fww
     */
    inline void setFullWaveformWriter(shared_ptr<FullWaveformWriter> fww)
    {this->fww = fww;}
    /**
     * @brief Validate the full waveform writer of the facade is valid to
     *  support write methods. If it is not valid, an adequate exception will
     *  be thrown.
     * @see FMSWriteFacade::fww
     */
    void validateFullWaveformWriter(
        string const &callerName="FMSWriteFacade::validateFullWaveformWriter",
        string const &errorMsg="could not accesss FullWaveformWriter"
    ) const;
    /**
     * @see FullWaveformWritter::writeFullWaveform
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
     * @brief Write the full waveform without validations (it is faster than
     *  its non unsafe counterpart)
     * @see FullWaveformWriter::writeFullWaveformUnsafe
     */
    inline void writeFullWaveformUnsafe(
        vector<double> const &fullwave,
        int const fullwaveIndex,
        double const minTime,
        double const maxTime,
        glm::dvec3 const &beamOrigin,
        glm::dvec3 const &beamDir,
        double const gpsTime
    ) const{
        fww->writeFullWaveformUnsafe(
            fullwave,
            fullwaveIndex,
            minTime,
            maxTime,
            beamOrigin,
            beamDir,
            gpsTime
        );
    }
    /**
     * @see filems::HeliosWriter::finish
     */
    void finishFullWaveformWriter();
    /**
     * @see filems::FullWaveformWriter::isZipOutput
     */
    bool isFullWaveformWriterZipOutput() const;
    /**
     * @see filems::FullWaveformWriter::setZipOutput
     */
    void setFullWaveformWriterZipOutput(bool const zipOutput);
};

    }}