#ifdef DATA_ANALYTICS

#include <dataanalytics/HDA_Recorder.h>
#include <util/HeliosException.h>

#include <boost/filesystem.hpp>

#include <sstream>


using namespace helios::analytics;

// ***  RECORDER METHODS  *** //
// ************************** //
void HDA_Recorder::validateOutDir(){
    // Check directory exists
    if(!boost::filesystem::exists(outdir)){
        if(!boost::filesystem::create_directory(outdir)){
            std::stringstream ss;
            ss  << "HDA_SimStepRecorder::validateOutDir thrown an exception "
                << "because the output directory does not exist and cannot be"
                << "created.\noutdir: \"" << outdir << "\"";
            throw HeliosException(ss.str());
        }
    }
}

std::string HDA_Recorder::craftOutputPath(std::string const &fname){
    std::stringstream ss;
    ss  << outdir
        << boost::filesystem::path::preferred_separator
        << fname;
    return ss.str();
}

#endif
