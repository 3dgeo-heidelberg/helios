#include <BaseMeasurementWriter.h>

using std::stringstream;
using std::ofstream;

// ***   M E T H O D S   *** //
// ************************* //
template <typename ... WriteArgs>
void BaseMeasurementWriter<WriteArgs ...>::configure(
    string const &parent,
    string const &prefix,
    bool const lastLegInStrip
){
    stringstream ss;
    ss << parent << prefix;
    if(this->isLasOutput()){
        if(isZipOutput()) ss << "_points.laz";
        else ss << "_points.las";
    }
    else if(isZipOutput()) ss << "_points.bin";
    else ss << "_points.xyz";
    setOutputFilePath(ss.str(), lastLegInStrip);
}

template <typename ... WriteArgs>
WriterType BaseMeasurementWriter<WriteArgs ...>::chooseWriterType() const {
    if(isLas10()) return las10Type;
    else if(isLasOutput()) return las14Type;
    else if(isZipOutput()) return zipType;
    return simpleType;
}

template <typename ... WriteArgs>
void BaseMeasurementWriter<WriteArgs ...>::clearPointcloudFile(){
    string outputPath = getOutputPath();
    logging::INFO("Clearing point cloud: \""+outputPath+"\"");
    ofstream ofs;
    try{
        ofs.open(outputPath, ofstream::out | ofstream::trunc);
    }
    catch(std::exception &ex){
        logging::ERR(ex.what());
    }
    ofs.close();
}


// ***  GETTERs and SETTERs  *** //
// ***************************** //
// ATTENTION: This method needs to be synchronized since multiple threads are
// writing to the output file!
template <typename ... WriteArgs>
void BaseMeasurementWriter<WriteArgs ...>::setOutputFilePath(
    string const &path,
    bool const lastLegInStrip
){
    logging::WARN("outputFilePath=" + path);
    try {
        WriterType wt = chooseWriterType();

        // Finish previous writer properly, if any
        if(sfw != nullptr) sfw->finish();

        // Create the Writer
        if(!fs::exists(path)){
            sfw = makeWriter(
                wt,                                     // Writer type
                path,                                   // Output path
                isZipOutput(),                          // Zip flag
                getLasScale(),                          // Scale factor
                shift,                                  // Offset
                0.0,                                    // Min intensity
                1000000.0                               // Delta intensity
            );
            writers[path] = sfw;
        }
        else{ // Consider existing writer
            sfw = writers[path];
        }

        // Remove writer from writers hashmap if it is the last leg in strip
        // to allow the sfw destructor to be called when sfw is replaced in the
        // next leg
        if(lastLegInStrip) writers.erase(path);

    } catch (std::exception &e) {
        logging::WARN(e.what());
    }
}
