// ***   M E T H O D S   *** //
// ************************* //
template <typename ... WriteArgs>
void BasePulseWriter<WriteArgs ...>::configure(
    std::string const &parent,
    std::string const &prefix,
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
    std::shared_ptr<SyncFileWriter<WriteArgs ...>> current = sfw;
    HeliosWriter<WriteArgs ...>::finish();

    // Finish remaining writers
    typename std::unordered_map<
        std::string, std::shared_ptr<SyncFileWriter<WriteArgs ...>>
    >::iterator it;
    for(it = writers.begin() ; it != writers.end() ; ++it){
        std::shared_ptr<SyncFileWriter<WriteArgs ...>> w = it->second;
        if(w!=current) w->finish();
    }
}

// ***   GETTERs and SETTERs   *** //
// ******************************* //
template <typename ... WriteArgs>
void BasePulseWriter<WriteArgs ...>::setOutputFilePath(
    std::string const &path
){
    logging::INFO("Pulses are written to: \"" + path + "\"");
    sfw = makeWriter(path);
}
