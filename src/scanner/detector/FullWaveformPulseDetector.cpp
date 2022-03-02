#include <iostream>
#include <exception>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <logging.hpp>
namespace fs = boost::filesystem;

#include "FullWaveformPulseRunnable.h"
#include "FullWaveformPulseDetector.h"

using namespace std;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
std::shared_ptr<AbstractDetector> FullWaveformPulseDetector::clone(){
    std::shared_ptr<AbstractDetector> fwpd =
        std::make_shared<FullWaveformPulseDetector>(
            scanner,
            cfg_device_accuracy_m,
            cfg_device_rangeMin_m
        );
    _clone(fwpd);
    return fwpd;
}
void FullWaveformPulseDetector::_clone(std::shared_ptr<AbstractDetector> ad){
    AbstractDetector::_clone(ad);
    FullWaveformPulseDetector *fwpd = (FullWaveformPulseDetector *) ad.get();
    fwpd->fw_sfw = fw_sfw;
}

// ***  M E T H O D S  *** //
// *********************** //
void FullWaveformPulseDetector::applySettings(shared_ptr<ScannerSettings> & settings) {
	AbstractDetector::applySettings(settings); // calls empty function
}

void FullWaveformPulseDetector::setOutputFilePath(
    std::string path,
    std::string fname,
    bool computeWaveform,
    bool lastLegInStrip
) {
    fms->write.setMeasurementWriterOutputFilePath(path, lastLegInStrip);

	if(computeWaveform) {
        try {
            std::string fw_path =
                fms->write.getMeasurementWriterOutputFilePath()
                .parent_path().parent_path()
                .string() + "/" + fname;
            logging::INFO("fw_path="+fw_path);
            if(fms->write.isMeasurementWriterZipOutput()){
                this->fw_sfw = std::make_shared<ZipSyncFileWriter>(fw_path);
            }
            else {
                this->fw_sfw = std::make_shared<SimpleSyncFileWriter>(
                    fw_path
                );
            }
        }
        catch (std::exception &e) {
            logging::INFO(e.what());
        }
    }
}

void FullWaveformPulseDetector::shutdown() {
	AbstractDetector::shutdown();
	if(fw_sfw != nullptr) fw_sfw->finish();
}

void FullWaveformPulseDetector::writeFullWave(
	vector<double> & fullwave, 
	int fullwave_index, 
	double min_time, 
	double max_time, 
	glm::dvec3& beamOrigin,
	glm::dvec3& beamDir,
    double gpstime
){
    if(fw_sfw != nullptr) {
        fw_sfw->write(
            fullwave,
            fullwave_index,
            min_time,
            max_time,
            beamOrigin,
            beamDir,
            gpstime
        );
    }
}