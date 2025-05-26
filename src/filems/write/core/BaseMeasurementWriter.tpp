// ***   M E T H O D S   *** //
// ************************* //
template <typename ... WriteArgs>
void BaseMeasurementWriter<WriteArgs ...>::configure(
    std::string const &parent,
    std::string const &prefix,
    bool const lastLegInStrip
){
    // Prepare
    std::stringstream ss;
    ss << parent << prefix;
    if(this->isLasOutput()){
        if(isZipOutput()) ss << "_points.laz";
        else ss << "_points.las";
    }
    else if(isZipOutput()) ss << "_points.bin";
    else ss << "_points.xyz";
    std::string const outpath = ss.str();

    // Log for debug level
    std::stringstream ss2;
    ss2     << "Parent path for base measurement writer: \""
            << parent << "\"\n"
            << "Prefix for base measurement writer: \""
            << prefix << "\"\n"
            << "Output path for base measurement writer: \""
            << outpath << "\"";
    logging::DEBUG(ss2.str());

    // Set
    setOutputFilePath(outpath, lastLegInStrip);
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
    std::string outputPath = getOutputPath();
    logging::INFO("Clearing point cloud: \""+outputPath+"\"");
    std::ofstream ofs;
    try{
        ofs.open(outputPath, std::ofstream::out | std::ofstream::trunc);
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
    std::shared_ptr<SyncFileWriter<WriteArgs ...>> current = sfw;
    HeliosWriter<WriteArgs ...>::finish();

    // Finish remaining writers
    typename std::unordered_map<
        std::string, std::shared_ptr<SyncFileWriter<WriteArgs ...>>
    >::iterator it;
    for(it = writers.begin() ; it != writers.end(); ++it){
        std::shared_ptr<SyncFileWriter<WriteArgs ...>> w = it->second;
        if(w!=current) w->finish();
    }
}


// ***  GETTERs and SETTERs  *** //
// ***************************** //
// ATTENTION: This method needs to be synchronized since multiple threads are
// writing to the output file!
template <typename ... WriteArgs>
void BaseMeasurementWriter<WriteArgs ...>::setOutputFilePath(
    std::string const &path,
    bool const lastLegInStrip
){
    logging::WARN("outputFilePath=" + path);
    try {
        WriterType wt = chooseWriterType();

        // Finish previous writer properly, if any
        //if(sfw != nullptr) sfw->finish(); // It is already done by destructor

        // Create the Writer
        if(!fs::exists(path)){
            logging::DEBUG("Creating writer for measurements ...");
            sfw = makeWriter(
                wt,                                     // Writer type
                path,                                   // Output path
                isZipOutput(),                          // Zip flag
                getLasScale(),                          // Scale factor
                shift,                                  // Offset
                0.0,                                    // Min intensity
                1000000.0                               // Delta intensity
            );
            logging::DEBUG("Created synchronous file writer!");
            writers[path] = sfw;
            logging::DEBUG("Stored synchronous file writer!");
        }
        else{ // Consider existing writer
            logging::DEBUG("Loading existing writer for measurements ...");
            sfw = writers[path];
        }

        // Remove writer from writers hashmap if it is the last leg in strip
        // to allow the sfw destructor to be called when sfw is replaced in the
        // next leg
        if(lastLegInStrip) {
            logging::DEBUG("Erasing existing writer ...");
            writers.erase(path);
        }

        std::stringstream ss;
        std::string const sfwPath = (sfw != nullptr) ? sfw->getPath() : "NULL";
        ss  << "Set output file path for measurements!\n"
            << "sfw = " << sfw << " writing to \"" << sfwPath << "\"";
        logging::DEBUG(ss.str());
    } catch (std::exception &e) {
        logging::WARN(e.what());
    }
}
