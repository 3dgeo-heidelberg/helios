// ***  READING METHODS  *** //
// ************************* //
template<typename VarType>
fluxionum::DesignMatrix<VarType>
helios::filems::DesignMatrixReader<VarType>::read(
  std::unordered_map<std::string, std::string>* keyval)
{
  // Prepare variables
  std::vector<std::string> header;
  std::vector<VarType> values;
  bool firstRow = true;
  std::size_t nValuesPerRow = 0;
  try {
    // Parsing loop : fill variables
    while (true) {
      std::string const str = br.read();
      std::size_t const comIdx = str.find(com);
      std::size_t const nonEmptyIdx = str.find_first_not_of(" \t");
      if (comIdx == nonEmptyIdx)
        parseComment(str, comIdx, header, keyval);
      else if (nonEmptyIdx != std::string::npos) {
        parseRow(str, nonEmptyIdx, values);
        if (firstRow) {
          nValuesPerRow = values.size();
          firstRow = false;
        }
      }
    }
  } catch (EndOfReadingException& eofrex) {
  } // Catch on end of reading
  // Build the DesignMatrix
  std::size_t const nRows = values.size() / nValuesPerRow;
  arma::Mat<VarType> X(nRows, nValuesPerRow);
  for (std::size_t i = 0; i < nRows; ++i) {
    std::size_t const rowOffset = i * nValuesPerRow;
    for (std::size_t j = 0; j < nValuesPerRow; ++j) {
      X(i, j) = values[rowOffset + j];
    }
  }
  // Return
  return fluxionum::DesignMatrix<VarType>(X, header);
}

// ***  PARSING UTILS  *** //
// *********************** //
template<typename VarType>
void
helios::filems::DesignMatrixReader<VarType>::parseComment(
  std::string const& str,
  std::size_t const comIdx,
  std::vector<std::string>& header,
  std::unordered_map<std::string, std::string>* keyval)
{
  std::size_t colonIdx;
  if (isSpecComment(str, colonIdx)) {
    std::string key, val;
    extractSpecCommentKeyValue(str, comIdx, colonIdx, key, val);
    if (key == "HEADER")
      parseColumnNames(val, header);
    else if (keyval != nullptr)
      keyval->emplace(key, val);
  }
}

template<typename VarType>
void
helios::filems::DesignMatrixReader<VarType>::parseRow(
  std::string const& str,
  std::size_t const nonEmptyIdx,
  std::vector<VarType>& values)
{
  // Make string s initially as str[nonEmptyIdx:] without spaces and tabs
  std::string s = str.substr(nonEmptyIdx);
  std::size_t i = 0;
  while (i < s.size()) {
    if (s[i] == ' ' || s[i] == '\t') {
      std::size_t j = i;
      while (j < s.size() && (s[j] == ' ' || s[j] == '\t'))
        ++j;
      s.erase(i, j - i);
    } else {
      ++i;
    }
  }

  // Extract values from row
  bool parsing = true;
  while (parsing) {
    std::size_t const sepIdx = s.find(sep);
    std::size_t const endIdx =
      (sepIdx == std::string::npos) ? s.size() : sepIdx;
    values.push_back((VarType)std::stod(s.substr(0, endIdx)));
    if (sepIdx == std::string::npos)
      parsing = false; // Stop condition
    else
      s = s.substr(endIdx + sep.size());
  }
}

template<typename VarType>
bool
helios::filems::DesignMatrixReader<VarType>::isSpecComment(
  std::string const& str,
  std::size_t& colonIdx)
{
  colonIdx = str.find(':');
  return colonIdx != std::string::npos;
}

template<typename VarType>
void
helios::filems::DesignMatrixReader<VarType>::extractSpecCommentKeyValue(
  std::string const& str,
  std::size_t const comIdx,
  std::size_t const colonIdx,
  std::string& key,
  std::string& val)
{
  key = str.substr(comIdx + com.size(), colonIdx - comIdx - com.size());
  val = str.substr(colonIdx + 1);
}

template<typename VarType>
void
helios::filems::DesignMatrixReader<VarType>::parseColumnNames(
  std::string const& val,
  std::vector<std::string>& header)
{
  bool insideName = false;
  std::size_t const nChars = val.size();
  std::size_t nameStart;
  for (std::size_t i = 0; i < nChars; ++i) {
    if (val[i] == '\"') {
      insideName = !insideName;
      if (insideName)
        nameStart = i;
      else
        header.push_back(val.substr(nameStart + 1, i - nameStart - 1));
    }
  }
}
