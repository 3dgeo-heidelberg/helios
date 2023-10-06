#include <BasePulseWriter.h>

using std::stringstream;
using std::ofstream;

// ***   M E T H O D S   *** //
// ************************* //
template <typename ... WriteArgs>
void BasePulseWriter<WriteArgs ...>::configure(
    string const &parent,
    string const &prefix,
    bool const writePulse
){
    // There is no need to configure output paths if there is no output at all
    if(!writePulse) return;

    // Configure output path
    std::stringstream ss;
    ss.str("");
    ss << parent << prefix;
    if(isZipOutput()) ss << "_pulse.bin";
    else ss << "_pulse.txt";
    setOutputFilePath(ss.str());
}

// ***  HELIOS WRITER METHODS  *** //
// ******************************* //
template <typename ... WriteArgs>
void BasePulseWriter<WriteArgs ...>::finish(){
    // Call parent finish method that finishes current writer
    shared_ptr<SyncFileWriter<WriteArgs ...>> current = sfw;
    HeliosWriter<WriteArgs ...>::finish();

    // Finish remaining writers
    typename unordered_map<
        string, shared_ptr<SyncFileWriter<WriteArgs ...>>
    >::iterator it;
    for(it = writers.begin() ; it != writers.end() ; ++it){
        shared_ptr<SyncFileWriter<WriteArgs ...>> w = it->second;
        if(w!=current) w->finish();
    }
}

// ***   GETTERs and SETTERs   *** //
// ******************************* //
template <typename ... WriteArgs>
void BasePulseWriter<WriteArgs ...>::setOutputFilePath(
    string const &path
){
    logging::INFO("Pulses are written to: \"" + path + "\"");
    sfw = makeWriter(path);
}
