#include "FullWaveformPulseRunnable.h"

#include "AbstractDetector.h"

#include <iostream>
using namespace std;

#include <logging.hpp>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
void AbstractDetector::_clone(std::shared_ptr<AbstractDetector> ad){
    ad->scanner = scanner; // Reference pointer => copy pointer, not object
	if(mBuffer == nullptr) ad->mBuffer = nullptr;
    else ad->mBuffer = std::make_shared<MeasurementsBuffer>(*mBuffer);
    if(sfw == nullptr) ad->sfw = nullptr;
	else ad->sfw = sfw;
    ad->cfg_device_accuracy_m = cfg_device_accuracy_m;
    ad->cfg_device_rangeMin_m = cfg_device_rangeMin_m;
    ad->outputFileLineFormatString = outputFileLineFormatString;
    ad->outputFilePath = outputFilePath;
    ad->lasOutput = lasOutput;
    ad->zipOutput = zipOutput;
}

// ***  M E T H O D S  *** //
// *********************** //
// ATTENTION: This method needs to be synchronized since multiple threads are writing to the output file!
void AbstractDetector::setOutputFilePath(string path) {
	this->outputFilePath = path;
	logging::WARN("outputFilePath="+path);
	try {
        fs::create_directories(outputFilePath.parent_path());
        if (lasOutput) {
            sfw = std::make_shared<LasSyncFileWriter>(
                path,                                   // Output path
                zipOutput,                              // Zip flag
                lasScale,                               // Scale factor
                scanner->platform->scene->getShift(),   // Offset
                0.0,                                    // Min intensity
                1000000.0                               // Delta intensity
            );
        }
        else if (zipOutput) {
            sfw = std::make_shared<ZipSyncFileWriter>(path);
        }
        else {
            sfw = std::make_shared<SimpleSyncFileWriter>(path);
        }
	}
	catch (std::exception &e) {
	    logging::WARN(e.what());
	}
}

void AbstractDetector::shutdown() {
    if(sfw != nullptr) sfw->finish();
}

void AbstractDetector::writeMeasurement(Measurement & m) {
	// Write measured point to output file
	if(sfw != nullptr) sfw->write(m, scanner->platform->scene->getShift());
}

void AbstractDetector::writeMeasurements(list<Measurement*> & ms) {
    // Write measured point to output file
	for (const Measurement* m : ms) {
		sfw->write(*m, scanner->platform->scene->getShift());
	}
}