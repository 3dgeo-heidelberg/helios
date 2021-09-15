#pragma once

// Includes
#include <string>
#include "LasSyncFileWriter.h"



/**
 * @author Miguel Yermo García
 * @version 1.0
 * @brief LasSyncFileWriter implementation for LAS v1.4 format
 */
class Las14SyncFileWriter : public LasSyncFileWriter
{
public:

  Las14SyncFileWriter();

  explicit Las14SyncFileWriter(
      const std::string &path_,
      bool compress_ = false,
      double scaleFactor_ = 0.0001,
      glm::dvec3 offset_ = glm::dvec3(0, 0, 0),
      double minIntensity_ = 0.0,
      double deltaIntensity_ = 1000000.0
  ) :
      LasSyncFileWriter() {
              path = path_;
              scaleFactor = scaleFactor_;
              offset = offset_;
              minIntensity = minIntensity_;
              deltaIntensity = deltaIntensity_;
              finished = false;

              // Craft header and point format
              craft();

              // Add extra attributes
              addExtraAttributes();

              // Create LASWriter
              createLasWriter(path, compress_);
          };

  // *** CRAFTING *** //
  /**
   * Crafting of header of the LAS file for version 1.4
   */
  void craft()
  {
    // Craft version of LasSyncWriter LAS 1.0
    LasSyncFileWriter::craft();

    // Update Header to 1.4 specification
    lwHeader.version_minor = U8(4);

    // Update Point Data Format to support new return number / classes
    lwHeader.point_data_format = 6;
    lwHeader.point_data_record_length = 30;

    // Adds the byte difference between LAS 1.4 and LAS 1.0 (350 - 227)
    lwHeader.header_size += 148;

    // Adds the byte difference to the data point offset
    lwHeader.offset_to_point_data += 148;

  }


};
