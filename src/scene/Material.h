#pragma once

#include <cmath>
#include <string>

/**
 * @brief Class representing a material specification
 */
class Material
{
  // *********************** //

public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Material name
   */
  std::string name = "default";
  /**
   * @brief Flag specifying if material is ground (true) or not (false)
   */
  bool isGround = false;
  /**
   * @brief Flag specifying if use vertex colors (true) or not (false).
   * Color usage is at the moment excluded from Helios++, so this attribute
   *  might be discarded in the future
   */
  bool useVertexColors = false;
  /**
   * @brief Path to the file containing material definition
   */
  std::string matFilePath;
  /**
   * @brief This attribute is currently not being used and might be discarded
   *  in the future
   */
  std::string map_Kd = "";
  /**
   * @brief Material reflectance
   */
  double reflectance = NAN;
  /**
   * @brief Material specularity
   */
  double specularity = 0;
  /**
   * @brief Material specular exponent
   */
  double specularExponent = 10;
  /**
   * @brief Material classification
   */
  int classification = 0;

  /**
   * @brief Material spectra, used to find material reflectance when using
   *  spectral library
   * @see SpectralLibrary
   * @see SpectralLibrary::setReflectances
   */
  std::string spectra = "";

  /** Use reflectance without angular modulation; overrides specularity. */
  bool isDirectionIndependent = false;
  Material() = default;
  Material(const Material&) = default;
  virtual ~Material() = default;
  /** Reject invalid specularity and exponent, including nonfinite values. */
  void validate() const;
};
