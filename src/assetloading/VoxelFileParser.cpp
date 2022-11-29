#include <VoxelFileParser.h>

char const VoxelFileParser::BLANK_CHARACTERS[] = {' ', '\t', '\r', '\n'};

// ***  P A R S E  *** //
// ******************* //
std::vector<std::shared_ptr<DetailedVoxel>> VoxelFileParser::parseDetailed(
    std::string const & path,
    size_t numHeaderLines,
    bool const exactFormat,
    bool const discardNullPad,
    std::string const separator
){
    std::vector<DetailedVoxel *> oldvec = bruteParseDetailed(
        path, numHeaderLines, exactFormat, discardNullPad, separator
    );
    std::vector<std::shared_ptr<DetailedVoxel>> newvec;
    for(DetailedVoxel *dv : oldvec){
        newvec.push_back(std::shared_ptr<DetailedVoxel>(dv));
    }
    return newvec;
}

std::vector<DetailedVoxel *> VoxelFileParser::bruteParseDetailed(
    std::string const & path,
    size_t numHeaderLines,
    bool const exactFormat,
    bool const discardNullPad,
    std::string const separator
){
    // Load voxels file
    std::vector<std::string> lines;
    loadFile(lines, path);

    // Prepare for parsing
    bool minCornerXFound=false, minCornerYFound=false, minCornerZFound=false,
        maxCornerXFound=false, maxCornerYFound=false, maxCornerZFound=false,
        splitXFound=false, splitYFound=false, splitZFound=false;
    double  minCornerX, minCornerY, minCornerZ,
            maxCornerX, maxCornerY, maxCornerZ;
    size_t splitX, splitY, splitZ;
    double voxelSize, maxPad;
    cleanLines(
        lines, numHeaderLines,
        minCornerXFound, minCornerX,
        minCornerYFound, minCornerY,
        minCornerZFound, minCornerZ,
        maxCornerXFound, maxCornerX,
        maxCornerYFound, maxCornerY,
        maxCornerZFound, maxCornerZ,
        splitXFound, splitX, splitYFound, splitY, splitZFound, splitZ,
        voxelSize, maxPad
    );

    // Check prepare is valid
    validateDetailed(
        minCornerXFound, minCornerYFound, minCornerZFound,
        maxCornerXFound, maxCornerYFound, maxCornerZFound,
        splitXFound, splitYFound, splitZFound
    );

    // Parse voxels
    std::vector<DetailedVoxel * > voxels(0);
    double voxelHalfSize = voxelSize / 2.0;
    const char * sep = separator.c_str();
    char format1[4096];
    sprintf(
        format1,
        "%s%s%s%s%s",
        "%zu",sep,"%zu",sep,"%zu"
    );
    char format2[4096];
    sprintf(
        format2,
        "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
        "%lf",sep,"%lf",sep,"%lf",sep,"%lf",sep,
        "%lf",sep,"%lf",sep,"%lf",sep,"%lf",sep,
        "%d",sep,"%d",sep,"%lf",sep,"%lf",sep,"%lf"
    );
    char format3[4096];
    sprintf(format3, "%s%s", "%lf", sep);
    DetailedVoxel *dv;
    for(std::string line : lines){
        dv = parseDetailedVoxelLine(
            line, separator, exactFormat, discardNullPad,
            format1, format2, format3,
            minCornerX, minCornerY, minCornerZ,
            maxCornerX, maxCornerY, maxCornerZ,
            voxelSize, voxelHalfSize, maxPad
        );
        if(dv != nullptr) voxels.push_back(dv);
    }

    // Return loaded voxels
    return voxels;
}
// ***  PARSING FUNCTIONS  *** //
// *************************** //
void VoxelFileParser::loadFile(
    std::vector<std::string> &lines,
    std::string const & path
){
    std::string line;

    try{
        std::ifstream inFile(path, std::ios::in);
        while(std::getline(inFile, line)){
            lines.push_back(line);
        }
        inFile.close();
    }catch(std::exception &ex){
        logging::ERR(std::string("VoxelFileParser::loadFile EXCEPTION:\n\t") +
            ex.what()
        );
        throw HeliosException("Could not load file '" + path + "'" );
    }

}

void VoxelFileParser::cleanLines(
    std::vector<std::string> &lines,
    size_t const numHeaderLines,
    bool &minCornerXFound,
    double &minCornerX,
    bool &minCornerYFound,
    double &minCornerY,
    bool &minCornerZFound,
    double &minCornerZ,
    bool &maxCornerXFound,
    double &maxCornerX,
    bool &maxCornerYFound,
    double &maxCornerY,
    bool &maxCornerZFound,
    double &maxCornerZ,
    bool &splitXFound,
    size_t &splitX,
    bool &splitYFound,
    size_t &splitY,
    bool &splitZFound,
    size_t &splitZ,
    double &voxelSize,
    double &maxPad
){
    bool remove;

    for(size_t i = 0 ; i < lines.size() ; i++){
        // By default, lines should not be removed
        remove = false;

        // Check if line should be removed
        if(isBlankLine(lines[i])) remove = true;
        else if(handleSpecLine(
            lines[i],
            minCornerXFound, minCornerX,
            minCornerYFound, minCornerY,
            minCornerZFound, minCornerZ,
            maxCornerXFound, maxCornerX,
            maxCornerYFound, maxCornerY,
            maxCornerZFound, maxCornerZ,
            splitXFound, splitX, splitYFound, splitY, splitZFound, splitZ,
            voxelSize, maxPad
        )) remove = true;

        // Remove line if necessary
        if(remove){
            lines.erase(lines.begin()+i);
            i--;
        }
        // Clean line as it was not removed
        else{
            cleanLine(lines[i]);
        }
    }

    removeHeaderLines(lines, numHeaderLines);
}

bool VoxelFileParser::isBlankLine(std::string const &line){
    bool hasBlankCharacter;
    for(size_t i = 0 ; i < line.size() ; i++){
        hasBlankCharacter = false;
        for(size_t j = 0 ; j < N_BLANK_CHARACTERS ; j++) {
            if (line[i] == BLANK_CHARACTERS[j]){
                hasBlankCharacter = true;
                break;
            }
        }
        if(!hasBlankCharacter) return false;
    }

    return true;
}

bool VoxelFileParser::handleSpecLine(
    std::string const &line,
    bool &minCornerXFound,
    double &minCornerX,
    bool &minCornerYFound,
    double &minCornerY,
    bool &minCornerZFound,
    double &minCornerZ,
    bool &maxCornerXFound,
    double &maxCornerX,
    bool &maxCornerYFound,
    double &maxCornerY,
    bool &maxCornerZFound,
    double &maxCornerZ,
    bool &splitXFound,
    size_t &splitX,
    bool &splitYFound,
    size_t &splitY,
    bool &splitZFound,
    size_t &splitZ,
    double &voxelSize,
    double &maxPad
){
    bool charIsBlank;

    for(size_t i = 0 ; i < line.size() ; i++){
        charIsBlank = false;
        for(size_t j = 0 ; j < N_BLANK_CHARACTERS ; j++){
            if(line[i] == BLANK_CHARACTERS[j]){
                charIsBlank = true;
                break;
            }
        }
        if(!charIsBlank){
            if(line[i] == SPEC_CHARACTER){
                handleSpec(
                    line,
                    minCornerXFound, minCornerX,
                    minCornerYFound, minCornerY,
                    minCornerZFound, minCornerZ,
                    maxCornerXFound, maxCornerX,
                    maxCornerYFound, maxCornerY,
                    maxCornerZFound, maxCornerZ,
                    splitXFound, splitX,
                    splitYFound, splitY,
                    splitZFound, splitZ,
                    voxelSize, maxPad
                );
                return true;
            }
            else return false;
        }
    }

    return false;
}

void VoxelFileParser::handleSpec(
    std::string const &line,
    bool &minCornerXFound,
    double &minCornerX,
    bool &minCornerYFound,
    double &minCornerY,
    bool &minCornerZFound,
    double &minCornerZ,
    bool &maxCornerXFound,
    double &maxCornerX,
    bool &maxCornerYFound,
    double &maxCornerY,
    bool &maxCornerZFound,
    double &maxCornerZ,
    bool &splitXFound,
    size_t &splitX,
    bool &splitYFound,
    size_t &splitY,
    bool &splitZFound,
    size_t &splitZ,
    double &voxelSize,
    double &maxPad
){
    size_t fieldNameStart=0, fieldNameEnd=0;
    for(size_t i = 0 ; i < line.size() ; i++){
        if(line[i]=='#') fieldNameStart = i+1;
        if(line[i]==':'){
            fieldNameEnd = i;
            break;
        }
    }

    std::string fieldName = line.substr(
        fieldNameStart,
        fieldNameEnd-fieldNameStart
    );
    std::string fieldValue = line.substr(fieldNameEnd+1);
    int n;

    if(fieldName == "min_corner"){
            n = sscanf(
            fieldValue.c_str(),
            "%lf %lf %lf",
            &minCornerX, &minCornerY, &minCornerZ
        );
        if(n>=1) minCornerXFound = true;
        if(n>=2) minCornerYFound = true;
        if(n>=3) minCornerZFound = true;
    }
    else if(fieldName == "max_corner"){
        n = sscanf(
            fieldValue.c_str(),
            "%lf %lf %lf",
            &maxCornerX, &maxCornerY, &maxCornerZ
        );
        if(n>=1) maxCornerXFound = true;
        if(n>=2) maxCornerYFound = true;
        if(n>=3) maxCornerZFound = true;
    }
    else if(fieldName == "split"){
        n = sscanf(
            fieldValue.c_str(),
            "%zu %zu %zu",
            &splitX, &splitY, &splitZ
        );
        if(n>=1) splitXFound = true;
        if(n>=2) splitYFound = true;
        if(n>=3) splitZFound = true;
    }
    else if(fieldName == "res"){
        n = sscanf(fieldValue.c_str(), "%lf", &voxelSize);

        // Also search for max_pad in the same line than res
        size_t i = fieldNameEnd+1;
        while(fieldName != "max_pad" && i < line.size()) {
            for (; i < line.size(); i++) {
                if(line[i]=='#') fieldNameStart = i+1;
                if(line[i]==':'){
                    fieldNameEnd = i;
                    i++;
                    break;
                }
            }
            fieldName = line.substr(
                fieldNameStart,
                fieldNameEnd-fieldNameStart
            );
            fieldValue = line.substr(fieldNameEnd+1);
        }

        // Consider max_pad
        if(fieldName == "max_pad"){
            n = sscanf(fieldValue.c_str(), "%lf", &maxPad);
        }

    }
}

void VoxelFileParser::cleanLine(std::string &line){
    bool forwardStop = false, backwardStop = false;
    size_t forwardEraseStart = 0;
    size_t backwardEraseStart = line.size()-1;
    size_t forwardEraseLen = 0;
    size_t backwardEraseLen = 0;
    bool foundBlankCharacter;
    for(size_t i = 0, j = line.size()-1; i < line.size() ; i++, j--){
        // Handle start to end cleaning (i)
        if(!forwardStop){
            foundBlankCharacter = false;
            for(size_t k = 0 ; k < N_BLANK_CHARACTERS ; k++){
                if(line[i] == BLANK_CHARACTERS[k]){
                    foundBlankCharacter = true;
                    break;
                }
            }
            if(foundBlankCharacter) forwardEraseLen++;
            else forwardStop = true;
        }

        // Handle end to start cleaning (j)
        if(!backwardStop){
            foundBlankCharacter = false;
            for(size_t k = 0 ; k < N_BLANK_CHARACTERS ; k++){
                if(line[j] == BLANK_CHARACTERS[k]){
                    foundBlankCharacter = true;
                    break;
                }
            }
            if(foundBlankCharacter) backwardEraseLen++;
            else backwardStop = true;
        }
    }

    line.erase(forwardEraseStart, forwardEraseLen);
    line.erase(
        backwardEraseStart-backwardEraseLen-forwardEraseLen+1,
        backwardEraseLen
    );
}

void VoxelFileParser::removeHeaderLines(
    std::vector<std::string> &lines,
    size_t numHeaderLines
){
    lines.erase(lines.begin(), lines.begin()+numHeaderLines);
}

void VoxelFileParser::validateDetailed(
    bool minCornerXFound,
    bool minCornerYFound,
    bool minCornerZFound,
    bool maxCornerXFound,
    bool maxCornerYFound,
    bool maxCornerZFound,
    bool splitXFound,
    bool splitYFound,
    bool splitZFound
){
    if(!minCornerXFound){
        throw HeliosException(
            "Can not parse detailed voxels because no min corner X was found"
        );
    }
    if(!minCornerYFound){
        throw HeliosException(
            "Can not parse detailed voxels because no min corner Y was found"
        );
    }
    if(!minCornerZFound){
        throw HeliosException(
            "Can not parse detailed voxels because no min corner Z was found"
        );
    }
    if(!maxCornerXFound){
        throw HeliosException(
            "Can not parse detailed voxels because no max corner X was found"
        );
    }
    if(!maxCornerYFound){
        throw HeliosException(
            "Can not parse detailed voxels because no max corner Y was found"
        );
    }
    if(!maxCornerZFound){
        throw HeliosException(
            "Can not parse detailed voxels because no max corner Z was found"
        );
    }
    if(!splitXFound){
        throw HeliosException(
            "Can not parse detailed voxels because no split X was found"
        );
    }
    if(!splitYFound){
        throw HeliosException(
            "Can not parse detailed voxels because no split Y was found"
        );
    }
    if(!splitZFound){
        throw HeliosException(
            "Can not parse detailed voxels because no split Z was found"
        );
    }
}

DetailedVoxel * VoxelFileParser::parseDetailedVoxelLine(
    std::string &line,
    std::string const separator,
    bool const exactFormat,
    bool const discardNullPad,
    char const *format1,
    char const *format2,
    char const *format3,
    double minCornerX,
    double minCornerY,
    double minCornerZ,
    double maxCornerX,
    double maxCornerY,
    double maxCornerZ,
    double voxelSize,
    double voxelHalfSize,
    double maxPad
){
    int n;
    std::vector<double> values;

    // Obtain voxel coordinates
    size_t i, j, k;
    n = sscanf(line.c_str(), format1, &i, &j, &k);
    if(n!=3){
        throw HeliosException(
            "VoxelFileParser::parseDetailedVoxelLine failed to parse params1"
        );
    }
    double x = minCornerX + i*voxelSize + voxelHalfSize;
    double y = minCornerY + j*voxelSize + voxelHalfSize;
    double z = minCornerZ + k*voxelSize + voxelHalfSize;
    for(int i = 0 ; i < n ; i++)
        line = line.substr(line.find_first_of(separator)+separator.length());

    // Parse values
    double pad,angMean,bsEnter,bsInterc,bsPoten,gDist,lMeanTotal,lgTotal;
    int nbEchos, nbSampl;
    double transmit, atten, attenBiasCorrec;
    n = sscanf(
        line.c_str(),
        format2,
        &pad,&angMean,&bsEnter,&bsInterc, &bsPoten,&gDist,&lMeanTotal,&lgTotal,
        &nbEchos, &nbSampl,
        &transmit, &atten, &attenBiasCorrec
    );
    if(n!=13){
        throw HeliosException(
            "VoxelFileParser::parseDetailedVoxelLine failed to parse params2"
        );
    }
    // Discard when PadBVTotal==0, if requested (e.g., transmittive mode)
    if(discardNullPad && pad==0) return nullptr;
    values.push_back(pad);          values.push_back(angMean);
    values.push_back(bsEnter);      values.push_back(bsInterc);
    values.push_back(bsPoten);      values.push_back(gDist);
    values.push_back(lMeanTotal);   values.push_back(lgTotal);
    values.push_back(transmit);     values.push_back(atten);
    values.push_back(attenBiasCorrec);

    if(!exactFormat) {
        // Prepare for parsing extra values
        double val;
        for (int i = 0; i < n-1; i++)
            line =  line.substr(
                line.find_first_of(separator) + separator.length()
            );

        // If there are values remaining ...
        size_t idx = line.find_first_of(separator);
        if(idx!=std::string::npos){
            line = line.substr(idx+separator.length());

            // Parse extra values
            while (true) {
                sscanf(line.c_str(), format3, &val);
                values.push_back(val);
                idx = line.find_first_of(separator);
                if(idx==std::string::npos) break;
                else line = line.substr(idx+separator.length());
            }
            /*
             * The parsing process here coded assumes not all rows need to have
             * the same number of extra elements.
             * Performance could be improved by assuming all rows will have
             * as many extra values as first row.
             *
             * ALSO reconsider reimplementing parseDetailedVoxelLine function
             * in future if it becomes a bottleneck.
             *
             * At the moment where it was coded, and considering the file
             * size it was tested with, it was not necessary to do a
             * more efficient implementation.
             */
        }
    }

    // Build DetailedVoxel
    DetailedVoxel * dv = new DetailedVoxel(
        x, y, z,
        voxelHalfSize,
        std::vector<int>({nbEchos, nbSampl}),
        values
    );
    dv->setMaxPad(maxPad);

    // Return DetailedVoxel
    return dv;
}
