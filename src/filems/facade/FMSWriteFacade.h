#pragma once

#include <filems/write/core/VectorialMeasurementWriter.h>
#include <filems/write/core/TrajectoryWriter.h>
#include <filems/write/core/VectorialFullWaveformWriter.h>
#include <filems/write/core/VectorialPulseWriter.h>
#include <scanner/detector/FullWaveform.h>

#include <glm/glm.hpp>

#include <vector>
#include <string>
#include <memory>
#include <list>

namespace helios { namespace filems {

class FMSFacadeFactory;

using std::vector;
using std::string;
using std::shared_ptr;
using std::list;

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
     * @brief The vectorial writer for measurements
     * @see filems::VectorialMeasurementWriter
     */
    shared_ptr<VectorialMeasurementWriter> mw = nullptr;
    /**
     * @brief The writer for trajectories
     * @see filems::TrajectoryWriter
     */
    shared_ptr<TrajectoryWriter> tw = nullptr;
    /**
     * @brief The writer for full waveform
     * @see filems::VectorialFullWaveformWriter
     */
    shared_ptr<VectorialFullWaveformWriter> fww = nullptr;
    /**
     * @brief The writer for pulses
     * @see filems::VectorialPulseWriter
     */
    shared_ptr<VectorialPulseWriter> pw = nullptr;
    /**
     * @brief The root directory for output files.
     */
    string rootDir = "./";

    /**
     * @brief Whether the write facade has been configured to split the
     *  output by channel (true) or not (false).
     *
     * NOTE that this flag is for reading purposes only, i.e., updating the
     *  flag will not change the way the writers work.
     */
    bool splitByChannel;

    /**
     * @brief The output directory on top of which the root directory is
     * built (it is the ancestor of the root directory).
     *
     * NOTE that this string is for reading purposes only, i.e., updating the
     *  string will not change the way the writers work.
     *
     * @see helios::filems::FMSWriteFacade::rootDir
     * @see helios::filems::FMSWriteFacade::buildFacade
     */
    string outDir;

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
     * @brief Disconnect all components from the write facade
     * @see filems::FMSFacade::disconnect
     */
    virtual void disconnect();
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
        bool const writePulse,
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
    /**
     * @brief Check whether the output is split by channel.
     * @see filems::FMSWriteFacade::splitByChannel
     */
    inline bool isSplitByChannel() const {return splitByChannel;}
    /**
     * @brief Obtain the output directory that is an ancestor of the
     *  root directory.
     * @see filems::FMSWriteFacade::outDir
     */
    inline string getOutDir() const {return outDir;}

    // ***  FACADE MEASUREMENT WRITE METHODS  *** //
    // ****************************************** //
    /**
     * @brief Obtain the measurement writer of the write facade
     * @return The measurement writer of the write facade
     * @see FMSWriteFacade::mw
     */
    inline shared_ptr<VectorialMeasurementWriter> getMeasurementWriter() const
    {return this->mw;}
    /**
     * @brief Set the measurement writer of the write facade
     * @param mw New measurement writer for the write facade
     * @see FMSWriteFacade::mw
     */
    inline void setMeasurementWriter(shared_ptr<VectorialMeasurementWriter> mw)
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
     * @bsee VectorialMeasurementWritter::writeMeasurements
     */
    void writeMeasurements(vector<Measurement> const &measurements);
    /**
     * @brief Write the vector of measurements without validations (it is
     *  faster than its non unsafe counterpart)
     * @see VectorialMeasurementWritter::writeMeasurementsUnsafe
     */
    inline void writeMeasurementsUnsafe(
        vector<Measurement> const &measurements
    ) const{mw->writeMeasurementsUnsafe(measurements);}
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
    inline shared_ptr<VectorialFullWaveformWriter> getFullWaveformWriter()
    const {return this->fww;}
    /**
     * @brief Set the full waveform writer of the write facade
     * @param fww New full waveform writer for the write facade
     * @see FMSWriteFacade::fww
     */
    inline void setFullWaveformWriter(
        shared_ptr<VectorialFullWaveformWriter> fww
    ) {this->fww = fww;}
    /**
     * @brief Validate the full waveform writer of the facade is valid to
     *  support write methods. If it is not valid, an adequate exception will
     *  be thrown.
     * @see FMSWriteFacade::fww
     */
    void validateFullWaveformWriter(
        string const &callerName="FMSWriteFacade::validateFullWaveformWriter",
        string const &errorMsg="could not access FullWaveformWriter"
    ) const;
    /**
     * @see VectorialFullWaveformWritter::writeFullWaveforms
     */
    void writeFullWaveforms(
        vector<FullWaveform> const &fullWaveforms
    );
    /**
     * @brief Write the full waveform without validations (it is faster than
     *  its non unsafe counterpart)
     * @see VectorialFullWaveformWriter::writeFullWaveformsUnsafe
     */
    inline void writeFullWaveformsUnsafe(
        vector<FullWaveform> const &fullWaveforms
    ) const{fww->writeFullWaveformsUnsafe(fullWaveforms);}
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

    // ***  FACADE PULSE WRITE METHODS  *** //
    // ************************************ //
    /**
     * @brief Obtain the pulse writer of the write facade
     * @return The pulse writer of the write facade
     * @see FMSWriteFacade::pw
     */
    inline shared_ptr<VectorialPulseWriter> getPulseWriter() const
    {return this->pw;}
    /**
     * @brief Set the pulse writer of the write facade
     * @param pw New pulse writer for the write facade
     * @see FMSWriteFacade::pw
     */
    inline void setPulseWriter(shared_ptr<VectorialPulseWriter> pw)
    {this->pw = pw;}
    /**
     * @brief Check the pulse writer of the facade is valid to support
     *  write methods. If it is not valid, an adequate exception will be
     *  thrown.
     * @see FMSWriteFacade::pw
     */
    void validatePulseWriter(
        string const &callerName="FMSWriteFacade::validateWriteFacade",
        string const &errorMsg="could not access PulseWriter"
    ) const;
    /**
     * @see VectorialPulseWriter::writePulses
     */
    void writePulses(
        vector<PulseRecord> const &pulses
    );
    /**
     * @brief Write the pulses without validations (it is faster than its non
     *  unsafe counterpart)
     * @see VectorialPulseWriter::writePulsesUnsafe
     */
    inline void writePulsesUnsafe(
        vector<PulseRecord> const &pulseRecords
    ) const {pw->writePulsesUnsafe(pulseRecords);}
    /**
     * @see filems::HeliosWriter::finish
     */
    void finishPulseWriter();
    /**
     * @see filems::HeliosWriter::isZipOutput
     */
    bool isPulseWriterZipOutput() const;
    /**
     * @see filems::HeliosWriter::setZipOutput
     */
    void setPulseWriterZipOutput(bool const zipOutput);
};

    }}