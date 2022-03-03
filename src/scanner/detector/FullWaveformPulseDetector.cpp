#include <iostream>
#include <exception>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "FullWaveformPulseRunnable.h"
#include "FullWaveformPulseDetector.h"
#include <filems/facade/FMSFacade.h>
#include <filems/write/ZipSyncFileWriter.h>
#include <logging.hpp>

using helios::filems::ZipSyncFileWriter;
using helios::filems::SimpleSyncFileWriter;

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

void FullWaveformPulseDetector::shutdown() {
	AbstractDetector::shutdown();
	if(fw_sfw != nullptr) fw_sfw->finish(); // TODO Rethink : To FMS
}