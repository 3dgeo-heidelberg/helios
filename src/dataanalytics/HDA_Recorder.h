#ifdef DATA_ANALYTICS
#pragma once

#include <string>

namespace helios {
namespace analytics {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief The HeliosDataAnalytics abstract recorder. It is, an abstract class
 *  that handle only the common baseline logic for any recorder.
 */
class HDA_Recorder
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Path to the output directory where the different outputs will be
   *  stored
   */
  std::string outdir;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Build a HDA_Recorder to write data files inside given output
   *  directory.
   * @param path Path to the output directory.
   */
  HDA_Recorder(std::string const& path)
    : outdir(path)
  {
    validateOutDir();
  }

  virtual ~HDA_Recorder() {}

  // ***  RECORDER METHODS  *** //
  // ************************** //
  /**
   * @brief Check whether the output directory is a valid one or not. If not,
   *  an exception will be thrown.
   *
   * This method can be overriden by derived classes when necessary, but it
   *  is not a must.
   */
  virtual void validateOutDir();

  /**
   * @brief Craft the full output path considering the output directory and
   *  the given file name
   * @param fname The given filename
   * @return Full output path considering the output directory and the given
   *  file name
   */
  virtual std::string craftOutputPath(std::string const& fname);
};

}
}

#endif
