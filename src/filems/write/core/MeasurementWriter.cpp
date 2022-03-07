#include <filems/write/core/MeasurementWriter.h>
#include <util/HeliosException.h>

#include <sstream>
#include <fstream>

using namespace helios::filems;

using std::stringstream;
using std::ofstream;

// ***   M E T H O D S   *** //
// ************************* //
void MeasurementWriter::configure(
    string const &parent,
    string const &prefix,
    bool const lastLegInStrip
){
    stringstream ss;
    ss << parent << prefix;
    if(isLasOutput()){
        if(isZipOutput()) ss << "_points.laz";
        else ss << "_points.las";
    }
    else if(isZipOutput()) ss << "_points.bin";
    else ss << "_points.xyz";
    setOutputFilePath(ss.str(), lastLegInStrip);
}
void MeasurementWriter::writeMeasurement(Measurement & m){
    // Check there is a sync file writer
    if(sfw == nullptr){
        throw HeliosException(
            "MeasurementWriter::writeMeasurement failed because there was no "
            "SyncFileWriter (sfw) available"
        );

    }
    // Check there is a scanner
    else if(scanner == nullptr){
        throw HeliosException(
            "MeasurementWriter::writeMeasurement failed because there was no "
            "Scanner to associate measurements with"
        );
    }

    // Write measured point to output file
    sfw->write(m, scanner->platform->scene->getShift());
}

void MeasurementWriter::writeMeasurements(list<Measurement*> & measurements){
    // Check that there is an available writer
    if(sfw == nullptr){
        throw HeliosException(
            "MeasurementWriter::writeMeasurements failed because there was no "
            "SyncFileWriter (sfw) available"
        );
    }
    // Check there is a scanner
    else if(scanner == nullptr){
        throw HeliosException(
            "MeasurementWriter::writeMeasurements failed because there was no "
            "Scanner to associate measurements with"
        );
    }

    // Write measured point to output file
    for (const Measurement* m : measurements) {
        sfw->write(*m, scanner->platform->scene->getShift());
    }
}

WriterType MeasurementWriter::chooseWriterType() const {
    // Get the type of writer to be created
    WriterType wt;
    if (lasOutput) wt = las10 ? las10Type : las14Type;
    else if (las10) wt = las10Type;
    else if (zipOutput) wt = zipType;
    else wt = simpleType;

    return wt;
}

void MeasurementWriter::clearPointcloudFile(){
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
void MeasurementWriter::setOutputFilePath(
    string const &path,
    bool const lastLegInStrip
){
    logging::WARN("outputFilePath=" + path);
    try {
        WriterType wt = chooseWriterType();

        // Create the Writer
        if(!fs::exists(path)){
            sfw = SyncFileMeasurementWriterFactory::makeWriter(
                wt,
                path,                                   // Output path
                isZipOutput(),                          // Zip flag
                getLasScale(),                          // Scale factor
                scanner->platform->scene->getShift(),   // Offset
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
