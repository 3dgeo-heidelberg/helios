#pragma once

#include <boost/asio/thread_pool.hpp>
#include <boost/asio/post.hpp>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "Scanner.h"
#include "ScannerSettings.h"

#include "Measurement.h"
#include "MeasurementsBuffer.h"
#include "SimpleSyncFileWriter.h"
#include "LasSyncFileWriter.h"
#include "Las14SyncFileWriter.h"
#include "ZipSyncFileWriter.h"
#include "SyncFileWriterFactory.h"

/**
 * @brief Base abstract class for detectors
 */
class AbstractDetector {
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Scanner which the detector belongs to
     */
	std::shared_ptr<Scanner> scanner;
	/**
	 * @brief Buffer to store measurements
	 */
	std::shared_ptr<MeasurementsBuffer> mBuffer;
	/**
	 * @brief Synchronous file writer
	 */
	std::shared_ptr<SyncFileWriter> sfw;

	/**
	 * @brief Detector accuracy in meters
	 */
	double cfg_device_accuracy_m = 0;
	/**
	 * @brief Minimum range for detector in meters
	 */
	double cfg_device_rangeMin_m = 0;

	// File output:
	/**
	 * @brief Flag specifying if detector output must be written in LAS
	 * format (true) or not (false)
	 * @see AbstractDetector::lasScale
	 */
	bool lasOutput;
        /**
         * @brief Flag specifying if detect output must be writing in LAS 1.0
         * (LAS 1.4 is written by default)
         */
        bool las10;
	/**
	 * @brief Flag specifying if detector output must be zipped (true)
	 * or not (false)
	 */
	bool zipOutput;
	/**
	 * @brief Scale factor specification to be used when LAS output format
	 * specified
	 * @see AbstractDetector::lasOutput
	 */
	double lasScale;

	/**
	 * @brief Format string for output file line
	 *
	 * No longer used since synchronous file writers are now responsible of
	 * handling output writing
	 */
	std::string outputFileLineFormatString =
	    "%.3f %.3f %.3f %.4f %.4f %d %d %d %s %d";

	/**
	 * @brief Path to output file
	 */
	fs::path outputFilePath;

	// ***  CONSTRUCTION / DESTRUCTION  *** //
	// ************************************ //
	/**
	 * @brief Base constructor for abstract detector
	 * @see AbstractDetector::scanner
	 * @see AbstractDetector::accuracy_m
	 * @see AbstractDetector::rangeMin_m
	 */
	AbstractDetector(
	    std::shared_ptr<Scanner> scanner,
	    double accuracy_m,
	    double rangeMin_m
    ){
	    this->lasOutput = false;
            this->las10     = false;
	    this->zipOutput = false;
	    this->lasScale = 0.0001;
            this->cfg_device_accuracy_m = accuracy_m;
            this->cfg_device_rangeMin_m = rangeMin_m;
            this->scanner = std::move(scanner);
	}
	virtual ~AbstractDetector() {}
	virtual std::shared_ptr<AbstractDetector> clone() = 0;
	virtual void _clone(std::shared_ptr<AbstractDetector> ad);

	// ***  M E T H O D S  *** //
	// *********************** //
	/**
	 * @brief Shutdown the detector when simulation has finished
	 */
	virtual void shutdown();
	/**
	 * @brief Write a measurement
	 * @param m Measurement to be written
	 */
	void writeMeasurement(Measurement & m);
	/**
	 * @brief Write a list of measurements
	 * @param m List of measurements to be written
	 */
        void writeMeasurements(std::list<Measurement*> & m);
        /**
         * @brief Choose a type of file writer based on input flags
         * @return Type of writer to be created
         */
        WriterType chooseWriterType();
        /**
         * @brief Apply scanner settings to the detector
         * @param settings Settings to be applied to de detector
         */
	virtual void applySettings(std::shared_ptr<ScannerSettings> & settings) {};

	/**
	 * @brief Compute pulse simulation
	 * @param pool Thread pool to be used to distribute computational burden
	 * @param absoluteBeamOrigin Origin of the beam in absolute coordinates
	 * @param absoluteBeamAttitude Attitude of the beam
	 * @param state_currentPulseNumber Current pulse number
	 * @param currentGpsTime Current GPS time
	 */
	virtual void simulatePulse(
	    thread_pool & pool,
	    glm::dvec3 absoluteBeamOrigin,
	    Rotation absoluteBeamAttitude,
	    int state_currentPulseNumber,
	    long currentGpsTime
    ) = 0;

	// ***  GETTERS and SETTERS  *** //
	// ***************************** //
    /**
     * @brief Set path to output file
     * @param path New path to output file
     */
    void setOutputFilePath(std::string path);
};