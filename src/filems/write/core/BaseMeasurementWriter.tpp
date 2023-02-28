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

// ***  HELIOS WRITER METHODS  *** //
// ******************************* //
template <typename ... WriteArgs>
void BaseMeasurementWriter<WriteArgs ...>::finish(){
    // Call parent finish method that finishes current writer
    shared_ptr<SyncFileWriter<WriteArgs ...>> current = sfw;
    HeliosWriter<WriteArgs ...>::finish();

    // Finish remaining writers
    typename unordered_map<
        string, shared_ptr<SyncFileWriter<WriteArgs ...>>
    >::iterator it;
    for(it = writers.begin() ; it != writers.end(); ++it){
        shared_ptr<SyncFileWriter<WriteArgs ...>> w = it->second;
        if(w!=current) w->finish();
    }
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
        //if(sfw != nullptr) sfw->finish(); // It is already done by destructor

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
