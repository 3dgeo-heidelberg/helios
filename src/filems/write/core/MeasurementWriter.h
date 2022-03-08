#pragma once

#include <util/HeliosException.h>
#include <filems/write/core/HeliosWriter.h>
#include <filems/factory/SyncFileMeasurementWriterFactory.h>
#include <scanner/Measurement.h>
#include <scanner/Scanner.h>

#include <boost/filesystem.hpp>

#include <string>
#include <memory>
#include <unordered_map>
#include <list>

namespace fs = boost::filesystem;

namespace helios { namespace filems {



using std::string;
using std::shared_ptr;
using std::unordered_map;
using std::list;


/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class to handle writing of measurements to generate HELIOS++ output
 *  virtual point clouds
 */
class MeasurementWriter :
    public HeliosWriter<Measurement const&, glm::dvec3 const&>
{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The scanner that generates the measurements to be written
     * @see filems::MeasurementWriter::setScanner
     */
    shared_ptr<Scanner> scanner = nullptr;
    /**
     * @brief The pointer to the shift vector to be applied to measurements
     * @see filems::MeasurementWriter::setScanner
     */
    glm::dvec3 shift;

    /**
     * @brief Map of writers. This map allows to reuse writers for legs grouped
     * in the same strip.
     */
    unordered_map<string, shared_ptr<SyncFileWriter<
        Measurement const&, glm::dvec3 const&
    >>> writers{};

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @bried Default constructor for measurement writer
     */
    MeasurementWriter() = default;
    virtual ~MeasurementWriter() = default;

    // ***   M E T H O D S   *** //
    // ************************* //
    /**
     * @brief Configure the output path for the measurement writer
     * @param parent Path to output directory for measurements files
     * @param prefix Prefix for the name of the output file
     * @param lastLegInStrip Specify whether the last leg belonged to a strip
     *  (true) or not (false)
     */
    void configure(
        string const &parent,
        string const &prefix,
        bool const lastLegInStrip
    );
    /**
	 * @brief Write a measurement
	 * @param m Measurement to be written
	 */
    void writeMeasurement(Measurement const &m);
    /**
     * @brief Like filems::MeasurementWriter::writeMeasurement but faster
     *  because there is no validation
     * @see filems::MeasurementWriter::writeMeasurement
     */
    inline void writeMeasurementUnsafe(Measurement const &m) const
    {sfw->write(m, shift);}
    /**
	 * @brief Write a list of measurements
	 * @param m List of measurements to be written
	 */
    void writeMeasurements(list<Measurement*> const &measurements);
    /**
     * @brief Like filems::MeasurementWriter::writeMeasurements but faster
     *  because there is no validation
     * @see filems::MeasurementWriter::writeMeasurements
     */
    inline void writeMeasurementsUnsafe(
        list<Measurement *> const &measurements
    ) const
    {for(Measurement *m : measurements) sfw->write(*m, shift);}
    /**
     * @brief Choose a type of file writer based on input flags
     * @return Type of writer to be created
     */
    WriterType chooseWriterType() const;
    /**
     * @brief Clear point cloud file for current leg
     */
    void clearPointcloudFile();

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Set path to output file
     * @param path New path to output file
     */
    void setOutputFilePath(string const &path, bool const lastLegInStrip);
    /**
     * @brief Obtain the scanner associated with the measurement writer
     * @see filems::MeasurementWriter::scanner
     */
    inline shared_ptr<Scanner> getScanner() const {return scanner;}
    /**
     * @brief Associate a new scanner with the measurement writter, which
     *  implies updating the shift vector to be the same than the one
     *  defined for the scene associated to the scanner
     * @see filems::MeasurementWriter::scanner
     * @see filems::MeasurementWriter::shift
     */
    inline void setScanner(shared_ptr<Scanner> scanner){
        this->scanner = scanner;
        this->shift = this->scanner->platform->scene->getShift();
    }
};

}}