#pragma once

#include <filems/write/SyncFileWriter.h>
#include <filems/factory/SyncFileWriterFactory.h>
#include <scanner/Measurement.h>
#include <scanner/Scanner.h>

#include <boost/filesystem.hpp>

#include <string>
#include <memory>
#include <unordered_map>
#include <list>

namespace fs = boost::filesystem;

namespace helios { namespace filems {



using ::std::string;
using ::std::shared_ptr;
using ::std::unordered_map;
using ::std::list;


/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class to handle writing of measurements to generate HELIOS++ output
 *  virtual point clouds
 */
class MeasurementWriter{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
	 * @brief Synchronous file writer
     * @see filems::SyncFileWriter
	 */
    shared_ptr<SyncFileWriter> sfw = nullptr;

    /**
     * @brief The scanner that generates the measurements to be written
     */
    shared_ptr<Scanner> scanner = nullptr;

    /**
	 * @brief Flag specifying if detector output must be written in LAS
	 * format (true) or not (false)
	 * @see helios::filems::MeasurementWriter::lasScale
	 */
    bool lasOutput = false;
    /**
     * @brief Flag specifying if detect output must be writing in LAS 1.0
     * (LAS 1.4 is written by default)
     */
    bool las10 = false;
    /**
	 * @brief Flag specifying if detector output must be zipped (true)
	 * or not (false)
	 */
    bool zipOutput = false;
    /**
	 * @brief Scale factor specification to be used when LAS output format
	 * specified
	 * @see helios::filems::MeasurementWriter::lasOutput
	 */
    double lasScale = 0.0001;

    /**
	 * @brief Format string for output file line
	 *
	 * No longer used since synchronous file writers are now responsible of
	 * handling output writing
	 */
    string outputFileLineFormatString =
        "%.3f %.3f %.3f %.4f %.4f %d %d %d %s %d";

    /**
	 * @brief Path to output file
	 */
    fs::path outputFilePath;
    /**
     * @brief Map of writers. This map allows to reuse writers for legs grouped
     * in the same strip.
     */
    unordered_map<string, shared_ptr<SyncFileWriter>> writers{};

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
	 * @brief Write a measurement
	 * @param m Measurement to be written
	 */
    void writeMeasurement(Measurement & m);
    /**
	 * @brief Write a list of measurements
	 * @param m List of measurements to be written
	 */
    void writeMeasurements(list<Measurement*> & measurements);
    /**
     * @brief Choose a type of file writer based on input flags
     * @return Type of writer to be created
     */
    WriterType chooseWriterType() const;
    /**
     * @brief Finish the measurement writer
     * @see filems::SyncFileWriter::finish
     */
    void finish();

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Get the path to the output file
     * @return The path to the output file
     */
    inline fs::path getOutputFilePath() const {return outputFilePath;}
    /**
     * @brief Set path to output file
     * @param path New path to output file
     */
    void setOutputFilePath(string path, const bool lastLegInStrip);
    /**
     * @brief Get the LAS output flag
     * @see filems::MeasurementWriter::lasOutput
     */
    inline bool isLasOutput() const {return lasOutput;}
    /**
     * @brief Set the LAS output flag
     * @see filems::MeasurementWriter::lasOutput
     */
    inline void setLasOutput(bool const lasOutput) {this->lasOutput = lasOutput;}
    /**
     * @brief Get the LAS 10 specification flag
     * @see filems::MeasurementWriter::las10
     */
    inline bool isLas10() const {return las10;}
    /**
     * @brief Set the LAS output flag
     * @see filems::MeasurementWriter::las10
     */
    inline void setLas10(bool const las10) {this->las10 = las10;}
    /**
     * @brief Get the zip output flag
     * @see filems::MeasurementWriter::zipOutput
     */
    inline bool isZipOutput() const {return zipOutput;}
    /**
     * @brief Set the zip output flag
     * @see filems::MeasurementWriter::zipOutput
     */
    inline void setZipOutput(bool const zipOutput)
    {this->zipOutput = zipOutput;}
    /**
     * @brief Obtain the LAS scale of the measurement writer
     * @see filems::MeasurementWriter::lasScale
     */
    inline double getLasScale() const {return lasScale;}
    /**
     * @brief Set the LAS scale of the measurement writer
     * @see filems::MeasurementWriter::lasScale
     */
    inline void setLasScale(double const lasScale)
    {this->lasScale = lasScale;}
    /**
     * @brief Obtain the scanner associated with the measurement writer
     * @see filems::MeasurementWriter::scanner
     */
    inline shared_ptr<Scanner> getScanner() const {return scanner;}
    /**
     * @brief Associate a new scanner with the measurement writter
     * @see filems::MeasurementWriter::scanner
     */
    inline void setScanner(shared_ptr<Scanner> scanner)
    {this->scanner = scanner;}
};

}}