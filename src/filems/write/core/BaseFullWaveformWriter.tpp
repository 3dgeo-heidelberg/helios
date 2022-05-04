#include <BaseFullWaveformWriter.h>


using std::stringstream;
using std::ofstream;

// ***   M E T H O D S   *** //
// ************************* //
template <typename ... WriteArgs>
void BaseFullWaveformWriter<WriteArgs ...>::configure(
    string const &parent,
    string const &prefix,
    bool const computeWaveform
){
    // There is no need to configure output paths if there is no output at all
    if(!computeWaveform) return;

    // Configure output path
    stringstream ss;
    ss.str("");
    ss << parent << prefix;
    if(isZipOutput()) ss << "_fullwave.bin";
    else ss << "_fullwave.txt";
    setOutputFilePath(ss.str());
}

// ***  GETTERs and SETTERs  *** //
// ***************************** //
template <typename ... WriteArgs>
void BaseFullWaveformWriter<WriteArgs ...>::setOutputFilePath(
    string const &path
){
    logging::INFO("fw_path=" + path);
    sfw = makeWriter(path);
}
