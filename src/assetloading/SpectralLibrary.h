#pragma once

// Class for loading and applying the material reflectances [0,100] from the
// ASTER Spectral Library

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "Scene.h"
#include "typedef.h"
#include <map>
#include <string>

/**
 * @brief Class representing the spectral library
 */
class SpectralLibrary
{

private:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Asset directories
   */
  const std::vector<std::string> assetsDir;

  /**
   * @brief Spectra specification
   */
  const std::string spectra;
  /**
   * @brief Default reflectance value
   */
  const double defaultReflectance = 50.0;
  /**
   * @brief Reflectances retrieved from file
   */
  std::map<std::string, float> reflectanceMap;
  /**
   * @brief Wavelength
   *
   * \f[
   *  w_{um} = w_{m} \cdot 10^{6}
   * \f]
   */
  float wavelength_um = 0;

public:
  /**
   * @brief Spectral library constructor
   * @param wavelength_m Wavelength in meters \f$w_{m}\f$
   * @param spectra Path to spectra file
   */
  SpectralLibrary(float wavelength_m,
                  std::vector<std::string> assetsDir,
                  std::string spectra);

private:
  /**
   * @brief Reflectance interpolation function
   *
   * \f[
   *  \Delta_{w} = w_{1} - w_{0} \\
   *  s = w_{um} - w_{0} \\
   *  \Delta_{r} = r_{1} - r_{0} \\
   *  r = r_{0} + \frac{s \Delta_{r}}{\Delta_{w}}
   * \f]
   *
   * @return Interpolated reflectance
   */
  float interpolateReflectance(float w0, float w1, float r0, float r1);
  /**
   *
   * @param Assist readReflectances method reading spectra file
   * @see SpectralLibrary::readReflectances
   */
  void readFileAster(boost::filesystem::path path);

public:
  /**
   * @brief Read spectra file
   * @see SpectralLibrary::readFileAster
   */
  void readReflectances();
  /**
   * @brief Fill scene materials reflectance with available spectra data
   * @param scene Scene with materials which reflectance must be setted
   */
  void setReflectances(Scene* scene);

  /**
   * @brief Obtain the default reflectance of the spectral library.
   * @see SpectralLibrary::defaultReflectance
   */
  inline double getDefaultReflectance() { return defaultReflectance; }
};
