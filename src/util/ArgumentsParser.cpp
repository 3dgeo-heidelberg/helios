#include <ArgumentsParser.h>

#include <iostream>

// *** PUBLIC METHODS *** //
// ********************** //
bool ArgumentsParser::parseHelpRequest(){
    return findIndexOfArgument("-h") >= 0 ||
        findIndexOfArgument("--help") >= 0;
}

bool ArgumentsParser::parseTestRequest(){
    return findIndexOfArgument("--test") >= 0;
}

std::string ArgumentsParser::parseTestDir(){
    int index = findIndexOfArgument("--testDir");
    if(index >= 0){
        size_t idx = std::string(argv[index+1]).length()-1;
        if(argv[index+1][idx] == '/') return argv[index+1];
        return std::string(argv[index+1])+"/";
    }
    return "data/test/";
}

std::string ArgumentsParser::parseDemoRequest(){
    int index = findIndexOfArgument("--demo");
    if(index >= 0) return argv[index+1];
    return "NULL";
}

std::string ArgumentsParser::parseDemoSurveyPath(){
    int index = findIndexOfArgument("--demoSurvey");
    if(index >= 0) return argv[index+1];
    return "";
}

std::string ArgumentsParser::parseDemoAssetsPath(){
    int index = findIndexOfArgument("--demoAssets");
    if(index >= 0) return argv[index+1];
    return "";
}

std::string ArgumentsParser::parseSurveyPath() {
    if(argc < 2){
        std::cout << "Survey path as first argument is required!" << std::endl;
        exit(1);
    }
    return std::string(argv[1]);
}

std::string ArgumentsParser::parseAssetsPath(){
    int index = findIndexOfArgument("--assets");
    if(index >= 0){
        size_t idx = std::string(argv[index+1]).length()-1;
        if(argv[index+1][idx] == '/') return argv[index+1];
        return std::string(argv[index+1])+"/";
    }
    return "assets/";
}

std::string ArgumentsParser::parseOutputPath(){
    int index = findIndexOfArgument("--output");
    if(index >= 0){
        size_t idx = std::string(argv[index+1]).length()-1;
        if(argv[index+1][idx] == '/') return argv[index+1];
        return std::string(argv[index+1])+"/";

    }
    return "output/";
}

bool ArgumentsParser::parseWriteWaveform(){
    return findIndexOfArgument("--writeWaveform") >= 0;
}

bool ArgumentsParser::parseCalcEchowidth(){
    return findIndexOfArgument("--calcEchowidth") >= 0;
}

std::string ArgumentsParser::parseSeed(){
    int index = findIndexOfArgument("--seed");
    if(index >= 0){
        return std::string(argv[index+1]);
    }
    return "";
}

std::size_t ArgumentsParser::parseNJobs(){
    int index = findIndexOfArgument("-j");
    if(index < 0) index = findIndexOfArgument("--njobs");
    if(index < 0) index = findIndexOfArgument("--nthreads");
    if(index < 0) return 0;
    return std::stoul(argv[index+1]);
}

std::size_t ArgumentsParser::parseChunkSize(){
    int index = findIndexOfArgument("--chunkSize");
    if(index < 0) return 32;
    return std::stoul(argv[index+1]);
}

bool ArgumentsParser::parseDisablePlatformNoise(){
    return findIndexOfArgument("--disablePlatformNoise")>=0;
}

bool ArgumentsParser::parseDisableLegNoise(){
    return findIndexOfArgument("--disableLegNoise")>=0;
}

bool ArgumentsParser::parseRebuildScene(){
    return findIndexOfArgument("--rebuildScene")>=0;
}

void ArgumentsParser::parseLoggingVerbosity(){
    if(findIndexOfArgument("--silent")>=0) logging::makeSilent();
    else if(
        findIndexOfArgument("-q") >= 0 ||
        findIndexOfArgument("--quiet") >= 0
    ){
        logging::makeQuiet();
    }
    else if(
        findIndexOfArgument("-v2")>=0 ||
        findIndexOfArgument("-vv")>=0
    ){
        logging::makeVerbose2();
    }
    else if(findIndexOfArgument("-vt")>=0) logging::makeTime();
    else if(findIndexOfArgument("-v")>=0) logging::makeVerbose();
    else logging::makeDefault();
}

std::string ArgumentsParser::parseLoggingOutputMode(){
    if(findIndexOfArgument("--logFile")>=0) return "full";
    else if(findIndexOfArgument("--logFileOnly")>=0) return "file";
    else return "std_out";
}

bool ArgumentsParser::parseFullWaveNoise(){
    return findIndexOfArgument("--fullwaveNoise") >= 0;
}

bool ArgumentsParser::parseLasOutput(){
    return findIndexOfArgument("--lasOutput") >= 0;
}

bool ArgumentsParser::parseLas10(){
    return findIndexOfArgument("--las10") >= 0;
}

bool ArgumentsParser::parseZipOutput(){
    return findIndexOfArgument("--zipOutput") >= 0;
}

double ArgumentsParser::parseLasScale(){
    int index = findIndexOfArgument("--lasScale");
    if(index < 0) return 0.0001;
    return std::stod(argv[index+1]);
}

bool ArgumentsParser::parseUnzip(
    std::string *inputPath,
    std::string *outputPath
){
    int index = findIndexOfArgument("--unzip");
    if(index < 0) return false;

    *inputPath = std::string(argv[index+1]);
    *outputPath = std::string(argv[index+2]);
    return true;
}

bool ArgumentsParser::parseFixedIncidenceAngle(){
    return findIndexOfArgument("--fixedIncidenceAngle") >= 0;
}

int ArgumentsParser::parseKDTreeType(){
    int index = findIndexOfArgument("--kdt");
    if(index < 0) return 1;
    return std::atoi(argv[index+1]);
}

size_t ArgumentsParser::parseKDTreeJobs(){
    int index = findIndexOfArgument("--kdtJobs");
    if(index < 0) return 1;
    return std::stoul(argv[index+1]);
}

size_t ArgumentsParser::parseSAHLossNodes(){
    int index = findIndexOfArgument("--sahNodes");
    if(index < 0) return 21;
    return std::stoul(argv[index+1]);
}

// *** PRIVATE METHODS *** //
// *********************** //
int ArgumentsParser::findIndexOfArgument(std::string&& arg){
    for(int i = 1 ; i < argc ; i++){
        if(arg == argv[i]) return i;
    }
    return -1;
}
