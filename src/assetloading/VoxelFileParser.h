#pragma once

#include <DetailedVoxel.h>
#include <HeliosException.h>
#include <fstream>
#include <iostream>
#include <logging.hpp>
#include <string>
#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class for parsing voxel files.
 *
 * Current implementation is based on AMAPVox format
 */
class VoxelFileParser
{
protected:
  // ***  CONSTANTS  *** //
  // ******************* //
  /**
   * @brief Nummber of blank characters
   */
  static size_t constexpr N_BLANK_CHARACTERS = 4;
  /**
   * @brief Characters considered "blank"
   */
  static char const BLANK_CHARACTERS[N_BLANK_CHARACTERS];
  /**
   * @brief Character which starts spec lines
   */
  static char constexpr SPEC_CHARACTER = '#';

public:
  // ***  CONSTRUCTOR  *** //
  // ********************* //
  /**
   * @brief Build a VoxelFileParser
   */
  VoxelFileParser() {}

  // ***  P A R S E  *** //
  // ******************* //
  /**
   * @brief Parse detailed voxels specified at file located at
   * given path
   * @param path Where the detailed voxels specification file
   * is located
   * @param numHeaderLines Specify how many header lines are used at
   * given file. Comments and blank lines must not be considered for this
   * count.
   * @param exactFormat When true it is assumed that only standard parameters
   * should be considering for each voxel. Using this knowledge parsing
   * process can be accelerated.
   * @param discardNullPad When true, those voxels with a PadBvTotal of 0
   *  will be discarded (by default they are preserved)
   * @param separator The separator between different fields
   *
   * For instance, if 1st line is a header line, 2nd line is a comment line,
   * 3rd line is a header line and from 4th line all lines are content;
   * number of header lines should be specified as 2
   *
   *
   * @return Parsed detailed voxels
   */
  std::vector<std::shared_ptr<DetailedVoxel>> parseDetailed(
    std::string const& path,
    size_t numHeaderLines = 2,
    bool const exactFormat = true,
    bool const discardNullPad = false,
    std::string const separator = " ");

  /**
   * @brief Like parseDetailed but returns a vector a normla pointers instead
   * of shared pointers
   * @see parseDetailed(
   *  std::string const &, size_t, bool const, std::string const)
   */
  std::vector<DetailedVoxel*> bruteParseDetailed(
    std::string const& path,
    size_t numHeaderLines = 2,
    bool const exactFormat = true,
    bool const discardNullPad = false,
    std::string const separator = " ");

protected:
  // ***  PARSING FUNCTIONS  *** //
  // *************************** //
  /**
   * @brief Loads the file at given path
   * @param lines Where lines shall be placed
   * @param path Path to the file to be loaded
   */
  void loadFile(std::vector<std::string>& lines, std::string const& path);
  /**
   * @brief Clean unnecessary lines
   *
   * <ol>
   *  <li>All spaces at the beginning and end of each line are removed.</li>
   *  <li>All lines which first character is '#' are considered as comments,
   *  hence removed</li>
   * </ol>
   *
   * @param lines Lines that shall be cleaned
   * @param numHeaderLines Number of header lines to be cleaned
   */
  void cleanLines(std::vector<std::string>& lines,
                  size_t const numHeaderLines,
                  bool& minCornerXFound,
                  double& minCornerX,
                  bool& minCornerYFound,
                  double& minCornerY,
                  bool& minCornerZFound,
                  double& minCornerZ,
                  bool& maxCornerXFound,
                  double& maxCornerX,
                  bool& maxCornerYFound,
                  double& maxCornerY,
                  bool& maxCornerZFound,
                  double& maxCornerZ,
                  bool& splitXFound,
                  size_t& splitX,
                  bool& splitYFound,
                  size_t& splitY,
                  bool& splitZFound,
                  size_t& splitZ,
                  double& voxelSize,
                  double& maxPad);

  /**
   * @brief Check if the line is a blank line
   *
   * Blank lines are those lines which only contain spaces, tabulations,
   * carriage return or new line characters.
   *
   * @param line Line to be checked
   * @return True if line is a blank line, False otherwise
   * @see BLANK_CHARACTERS
   */
  bool isBlankLine(std::string const& line);
  /**
   * @brief Handle line if it is an spec line. Do nothing otherwise.
   *
   * Spec lines are so because its first non blank character is #
   *
   * @param line Line to be handled
   * @return True if line is a spec line, False otherwise
   * @see BLANK_CHARACTERS
   * @see SPEC_CHARACTER
   */
  bool handleSpecLine(std::string const& line,
                      bool& minCornerXFound,
                      double& minCornerX,
                      bool& minCornerYFound,
                      double& minCornerY,
                      bool& minCornerZFound,
                      double& minCornerZ,
                      bool& maxCornerXFound,
                      double& maxCornerX,
                      bool& maxCornerYFound,
                      double& maxCornerY,
                      bool& maxCornerZFound,
                      double& maxCornerZ,
                      bool& splitXFound,
                      size_t& splitX,
                      bool& splitYFound,
                      size_t& splitY,
                      bool& splitZFound,
                      size_t& splitZ,
                      double& voxelSize,
                      double& maxPad);

  /**
   * @brief Assuming received line is a spec line, handle it.
   *
   * Handling a spec line means received arguments will be modified
   * accordingly to what have been handled.
   *
   * @param line Spec line
   * @param minCornerXFound Specify if the X coordinate of the minimum
   * corner of the bounding box containing the voxels has been found (true)
   * or not (false)
   * @param minCornerX X coordinate of the minimum corner of the bounding
   * box containing the voxels.
   * @param minCornerYFound Specify if the Y coordinate of the minimum
   * corner of the bounding box containing the voxels has been found (true)
   * or not (false)
   * @param minCornerY Y coordinate of the minimum corner of the bounding
   * box containing the voxels.
   * @param minCornerZFound Specify if the Z coordinate of the minimum
   * corner of the bounding box containing the voxels has been found (true)
   * or not (false)
   * @param minCornerZ Z coordinate of the minimum corner of the bounding
   * box containing the voxels.
   * @param maxCornerXFound Specify if the X coordinate of the maximum
   * corner of the bounding box containing the voxels has been found (true)
   * or not (false)
   * @param maxCornerX X coordinate of the maximum corner of the bounding
   * box containing the voxels.
   * @param maxCornerYFound Specify if the Y coordinate of the maximum
   * corner of the bounding box containing the voxels has been found (true)
   * or not (false)
   * @param maxCornerY Y coordinate of the maximum corner of the bounding
   * box containing the voxels.
   * @param maxCornerZFound Specify if the Z coordinate of the maximum
   * corner of the bounding box containing the voxels has been found (true)
   * or not (false)
   * @param maxCornerZ Z coordinate of the maximum corner of the bounding
   * box containing the voxels.
   * @param splitXFound Specify if the number of voxels along the X axis
   * of the bounding box containing the voxels has been found (true) or not
   * (false)
   * @param splitX Number of voxels along the X axis of the bounding box
   * containing the voxels
   * @param splitYFound Specify if the number of voxels along the Y axis
   * of the bounding box containing the voxels has been found (true) or not
   * (false)
   * @param splitY Number of voxels along the Y axis of the bounding box
   * containing the voxels
   * @param splitZFound Specify if the number of voxels along the Z axis
   * of the bounding box containing the voxels has been found (true) or not
   * (false)
   * @param splitZ Number of voxels along the Z axis of the bounding box
   * containing the voxels
   * @param voxelSize The size for each voxel
   */
  void handleSpec(std::string const& line,
                  bool& minCornerXFound,
                  double& minCornerX,
                  bool& minCornerYFound,
                  double& minCornerY,
                  bool& minCornerZFound,
                  double& minCornerZ,
                  bool& maxCornerXFound,
                  double& maxCornerX,
                  bool& maxCornerYFound,
                  double& maxCornerY,
                  bool& maxCornerZFound,
                  double& maxCornerZ,
                  bool& splitXFound,
                  size_t& splitX,
                  bool& splitYFound,
                  size_t& splitY,
                  bool& splitZFound,
                  size_t& splitZ,
                  double& voxelSize,
                  double& maxPad);

  /**
   * @brief Remove unnecessary stuff from received line.
   *
   * All starting and ending spaces are removed
   *
   * @param line Line to be cleaned
   */
  void cleanLine(std::string& line);

  /**
   * @brief Remove header lines
   * @param lines From where header lines shall be removed
   * @param numHeaderLines How many header lines shall be removed
   */
  void removeHeaderLines(std::vector<std::string>& lines,
                         size_t numHeaderLines);

  /**
   * @brief Check if flags are valid
   * @throw HeliosException if there is a not valid flag
   */
  void validateDetailed(bool minCornerXFound,
                        bool minCornerYFound,
                        bool minCornerZFound,
                        bool maxCornerXFound,
                        bool maxCornerYFound,
                        bool maxCornerZFound,
                        bool splitXFound,
                        bool splitYFound,
                        bool splitZFound);

  /**
   * @brief Parse the DetailedVoxel specified in given line
   * @param line Line where the DetailedVoxel is specified
   * @param separator The separator between different fields
   * @param format1 Format string to parse indices at X,Y,Z (i,j,k), so
   * voxel coordinates can be computed
   * @param format2 Format string to parse expected doubles
   * @param format3 Format string to parse extra values (doubles)
   * @return Parsed DetailedVoxel (nullptr if it was discarded, for instance
   *  to ignore transmittive voxels with PadBVTotal==0)
   */
  DetailedVoxel* parseDetailedVoxelLine(std::string& line,
                                        std::string const separator,
                                        bool const exactFormat,
                                        bool const discardNullPad,
                                        char const* format1,
                                        char const* format2,
                                        char const* format3,
                                        double minCornerX,
                                        double minCornerY,
                                        double minCornerZ,
                                        double maxCornerX,
                                        double maxCornerY,
                                        double maxCornerZ,
                                        double voxelSize,
                                        double voxelHalfSize,
                                        double maxPad);
};
