// ***   M E T H O D S   *** //
// ************************* //
template <typename ... WriteArgs>
void BaseFullWaveformWriter<WriteArgs ...>::configure(
    std::string const &parent,
    std::string const &prefix,
    bool const computeWaveform
){
    // There is no need to configure output paths if there is no output at all
    if(!computeWaveform) return;

    // Configure output path
    std::stringstream ss;
    ss.str("");
    ss << parent << prefix;
    if(isZipOutput()) ss << "_fullwave.bin";
    else ss << "_fullwave.txt";
    setOutputFilePath(ss.str());
}

// ***  HELIOS WRITER METHODS  *** //
// ******************************* //
template <typename ... WriteArgs>
void BaseFullWaveformWriter<WriteArgs ...>::finish(){
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
template <typename ... WriteArgs>
void BaseFullWaveformWriter<WriteArgs ...>::setOutputFilePath(
    std::string const &path
){
    logging::INFO("fw_path=" + path);
    sfw = makeWriter(path);
}
