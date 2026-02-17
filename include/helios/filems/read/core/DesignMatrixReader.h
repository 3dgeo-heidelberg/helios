#ifndef _HELIOS_FILEMS_DESIGN_MATRIX_READER_H_
#define _HELIOS_FILEMS_DESIGN_MATRIX_READER_H_

#include <helios/filems/read/comps/BufferedLineFileReader.h>
#include <helios/filems/read/exceptions/EndOfReadingException.h>
#include <helios/util/HeliosException.h>

#include <armadillo>

#include <string>
#include <unordered_map>
#include <vector>

namespace fluxionum {
template<typename T>
class DesignMatrix;
}

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class to read design matrices
 *
 * A design matrix file is a CSV text file with 2 special type of string
 *  tokens. By default the separator token is the comma "," and the default
 *  comment token is the "#". Thus, it is possible to have 2 different types
 *  of lines.
 *
 * The first type of line is the comment line. Any line which first non space
 *  and non tab substring is a comment token, is a comment line. There are
 *  2 different types of comments. The typical comments, which are simply
 *  ignored during parsing and are useful for human understanding purposes
 *  only. And the specification comments that can be used to specify some extra
 *  information to the parser. Specification comments can be used to specify
 *  a header, such that:
 *
 * <p><i>#HEADER: "field1","field2","field3"</i></p>
 *
 * The second type of line is the data record, data row or data point. It is
 *  a line of \f$n\f$ separated fields each such that:
 * <p><i>0.5, 1.0, 2.1</i></p>
 *
 * @tparam VarType The type of value for the design matrix
 * @see fluxionun::DesignMatrix
 */
template<typename VarType>
class DesignMatrixReader
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The buffered line file reader to read the design matrix
   */
  BufferedLineFileReader br;
  /**
   * @brief The field separator
   */
  std::string sep;
  /**
   * @brief The comment string token . Any line which first non space and non
   *  tab substring matches the comment string, is a commented line
   */
  std::string com;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for DesignMatrixReader
   * @param path Path to the input file containing a Design Matrix
   * @param sep The field separator
   * @param maxCharsPerLine Maximum number of characters per line
   * @param bufferSize The buffer size in number of lines
   */
  DesignMatrixReader(std::string const& path,
                     std::string const& sep = ",",
                     std::string const& com = "#",
                     long const maxCharsPerLine = 8192,
                     std::size_t const bufferSize = 100000)
    : br(path,              // Path to the input file
         std::ios_base::in, // Open mode
         maxCharsPerLine,   // Max size per line
         bufferSize         // How many lines
         )
    , sep(sep)
    , com(com)
  {
  }
  virtual ~DesignMatrixReader() = default;

  // ***  READING METHODS  *** //
  // ************************* //
  /**
   * @brief Read the design matrix
   * @param[out] keyval If it is not null, then it will be used to store all
   *  key value pairs from specification comments (unless the header)
   * @return Built design matrix
   */
  virtual fluxionum::DesignMatrix<VarType> read(
    std::unordered_map<std::string, std::string>* keyval = nullptr);

protected:
  // ***  PARSING UTILS  *** //
  // *********************** //
  /**
   * @brief Parse a comment line
   * @param str The line being parsed
   * @param comIdx The index of the first character of the comment token
   *  in the line
   * @param[out] header The vector of column names defining the header.
   * @param[out] keyval If it is not null, then it will be used to store all
   *  key value pairs from specification comments (unless the header)
   * If the parsed comment is a header specification, then it will be filled
   */
  virtual void parseComment(
    std::string const& str,
    std::size_t const comIdx,
    std::vector<std::string>& header,
    std::unordered_map<std::string, std::string>* keyval);
  /**
   * @brief Parse a row
   * @param str The line being parsed
   * @param nonEmptyIdx The index of the first non empty character in the
   *  line
   * @param[out] values The vector of values defining the contents of the
   *  DesignMatrix
   */
  virtual void parseRow(std::string const& str,
                        std::size_t const nonEmptyIdx,
                        std::vector<VarType>& values);
  /**
   * @brief Check whether the comment string is a specification comment or
   *  not.
   *
   * The method assumes the string is in fact a comment, so it must be
   *  to work properly. It only checks whether it is a typical comment or a
   *  specification comment
   *
   * @param str The comment line being parsed. It <b>MUST</b> be a comment
   *  line. The method assumes it is a comment line and only checks whether
   *  it is a specification comment or not
   * @param[out] colonIdx The index of the colon separator will be stored
   *  here. When it is a specification comment it will be distinct than
   *  string::npos (true), otherwise it will be exactly string::npos (false)
   *
   * @return True if the string is a specification comment, false if the
   *  string is a typical comment
   */
  virtual bool isSpecComment(std::string const& str, std::size_t& colonIdx);
  /**
   * @brief Extract the key and value from given string representing a
   *  specification comment
   * @param str String representing the specification comment
   * @param[in] comIdx The index of the first character of the comment token
   *  in the line
   * @param[in] colonIdx The index of the colon separator between key and
   *  value
   * @param[out] key Where the extracted key will be stored
   * @param[out] val Where the extracted value will be stored
   */
  virtual void extractSpecCommentKeyValue(std::string const& str,
                                          std::size_t const comIdx,
                                          std::size_t const colonIdx,
                                          std::string& key,
                                          std::string& val);
  /**
   * @brief Parse the list of inline column names
   * @param val Inline list of column names. Each name must be separated
   *  by the separator character DesignMatrixReader::sep
   * @param[out] header Where the column names are stored
   * @see DesignMatrixReader::sep
   */
  virtual void parseColumnNames(std::string const& val,
                                std::vector<std::string>& header);
};

}
}

#include <helios/filems/read/core/DesignMatrixReader.tpp>

#endif
