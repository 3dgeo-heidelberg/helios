#include <filems/read/core/DesignMatrixReader.h>
#include <filems/read/exceptions/EndOfReadingException.h>
#include <fluxionum/DesignMatrix.h>
#include <util/HeliosException.h>

#include <string>
#include <vector>

using namespace helios::filems;
using namespace fluxionum;
using std::string;
using std::vector;

// ***  READING METHODS  *** //
// ************************* //
template <typename VarType>
DesignMatrix<VarType> DesignMatrixReader<VarType>::read(
    std::unordered_map<string, string> *keyval
){
    // Prepare variables
    vector<string> header;
    vector<VarType> values;
    bool firstRow = true;
    size_t nValuesPerRow = 0;
    try{
        // Parsing loop : fill varialbes
        while(true){
            string const str = br.read();
            size_t const comIdx = str.find(com);
            size_t const nonEmptyIdx = str.find_first_not_of(" \t");
            if(comIdx==nonEmptyIdx) parseComment(str, comIdx, header, keyval);
            else if(nonEmptyIdx!=string::npos){
                parseRow(str, nonEmptyIdx, values);
                if(firstRow){
                    nValuesPerRow = values.size();
                    firstRow = false;
                }
            }
        }
    }
    catch(EndOfReadingException &eofrex){} // Catch on end of reading
    // Build the DesignMatrix
    size_t const nRows = values.size() / nValuesPerRow;
    arma::Mat<VarType> X(nRows, nValuesPerRow);
    for(size_t i = 0 ; i < nRows ; ++i){
        size_t const rowOffset = i*nValuesPerRow;
        for(size_t j = 0 ; j < nValuesPerRow ; ++j){
            X(i, j) = values[rowOffset+j];
        }
    }
    // Return
    return DesignMatrix<VarType>(X, header);
}

// ***  PARSING UTILS  *** //
// *********************** //
template <typename VarType> void DesignMatrixReader<VarType>::parseComment(
    string const &str,
    size_t const comIdx,
    vector<string> &header,
    unordered_map<string, string> *keyval
){
    size_t colonIdx;
    if(isSpecComment(str, colonIdx)){
        string key, val;
        extractSpecCommentKeyValue(str, comIdx, colonIdx, key, val);
        if(key=="HEADER") parseColumnNames(val, header);
        else if(keyval != nullptr) keyval->emplace(key, val);
    }
}

template <typename VarType> void DesignMatrixReader<VarType>::parseRow(
    string const &str,
    size_t const nonEmptyIdx,
    vector<VarType> &values
){
    // Make string s initially as str[nonEmptyIdx:] without spaces and tabs
    string s = str.substr(nonEmptyIdx);
    size_t n = s.size();
    size_t eraseStart = string::npos;
    for(size_t i = 0 ; i < n ; ++i){
        // TODO Rethink : Ignore space or tab if they are the separator
        if(s[i]==' ' || s[i]=='\t'){
            if(eraseStart == string::npos) eraseStart = i;
        }
        else if(eraseStart != string::npos){
            s.erase(eraseStart, i-eraseStart);
            eraseStart = string::npos;
        }
    }

    // Extract values from row
    bool parsing = true;
    while(parsing){
        size_t const sepIdx = s.find(sep);
        size_t const endIdx = (sepIdx==string::npos) ? s.size() : sepIdx;
        values.push_back((VarType) std::stod(s.substr(0, endIdx)));
        if(sepIdx == string::npos) parsing=false; // Stop condition
        else s = s.substr(endIdx+sep.size());
    }
}

template <typename VarType> bool DesignMatrixReader<VarType>::isSpecComment(
    string const &str,
    size_t &colonIdx
){
    colonIdx = str.find(':');
    return colonIdx != string::npos;
}

template <typename VarType> void
DesignMatrixReader<VarType>::extractSpecCommentKeyValue(
    string const &str,
    size_t const comIdx,
    size_t const colonIdx,
    string &key,
    string &val
){
    key = str.substr(comIdx+com.size(), colonIdx-comIdx-com.size());
    val = str.substr(colonIdx+1);
}

template <typename VarType> void DesignMatrixReader<VarType>::parseColumnNames(
    string const &val,
    vector<string> &header
){
    bool insideName = false;
    size_t const nChars = val.size();
    size_t nameStart;
    for(size_t i = 0 ; i < nChars ; ++i){
        if(val[i]=='\"'){
            insideName = !insideName;
            if(insideName) nameStart = i;
            else header.push_back(val.substr(nameStart+1, i-nameStart-1));
        }
    }
}