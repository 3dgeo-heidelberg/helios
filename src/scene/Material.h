#pragma once

#include <cmath>
#include <string>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

/**
 * @brief Class representing a material specification
 */
class Material
{
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & name;
    ar & isGround;
    ar & useVertexColors;
    ar & matFilePath;
    ar & map_Kd;
    ar & reflectance;
    ar & specularity;
    ar & specularExponent;
    ar & classification;
    ar & spectra;
    ar & ka & kd & ks;
  }

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
   * @see Material::setSpecularity
   */
  double specularity = 0;
  /**
   * @brief Material specular exponent
   */
  double specularExponent = 10;
  /**
   * @brief Material classification
   * @see LasSpecification
   */
  int classification = 0;

  /**
   * @brief Material spectra, used to find material reflectance when using
   *  spectral library
   * @see SpectralLibrary
   * @see SpectralLibrary::setReflectances
   */
  std::string spectra = "";

  /**
   * @brief Material ambient components
   */
  float ka[4] = { 0, 0, 0, 0 };
  /**
   * @brief Material diffuse components
   */
  float kd[4] = { 0, 0, 0, 0 };
  /**
   * @brief Material specular components
   */
  float ks[4] = { 0, 0, 0, 0 };

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Material default constructor
   */
  Material() = default;
  Material(const Material& sp);
  virtual ~Material() {}

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @brief Obtain specular component.
   * @param factor Factor to be applied to diffuse components before
   *  returning
   * @return 3D float array containing the diffuse components. NOTICE it must
   *  be released once it is no longer necessary
   *
   * @code
   * // Obtain
   * float *diffuse = getKd(1.0);
   * // Do stuff here
   * ...
   * // Release
   * delete[] diffuse
   * @endcode
   */
  float* getKd(float factor);
  /**
   * @brief Compute the specularity \f$s\f$
   *
   * Let \f$Kd\f$ represent the diffuse components and \f$Ks\f$
   *  represent the specular components
   *
   * \f[
   *  s = \frac{\sum_{i=1}^{3}{Ks_{i}}}
   *      {\sum_{i=1}^{3}{Kd_{i}} + \sum_{i=1}^{3}{Ks_{i}}}
   * \f]
   *
   * @see Material::specularity
   */
  void setSpecularity();
  /**
   * @brief Find if the \f$\pmb{K}_s\f$ and \f$\pmb{K}_d\f$ vectors have
   *  any non-null component or not.
   * @param[out] nonNullKs Will be true if vector \f$\pmb{K}_s\f$ has
   *  at least one non-null component, false otherwise.
   * @param[out] nonNullKd Will be true if vector \f$\pmb{K}_d\f$ has
   *  at least one non-null component, false otherwise.
   */
  void findNonNullComponents(bool& nonNullKs, bool& nonNullKd) const;
  /**
   * @brief A material is said to be Phong when it has both specular
   *  \f$\pmb{K}_s\f$ and diffuse \f$\pmb{K}_d\f$ vectors with at least one
   *  non-null component.
   * @return True if the material is Phong (specular and diffuse), False
   *  otherwise.
   */
  bool isPhong() const;
  /**
   * @brief A material is said to be Lambert when it has a null specular
   *  vector \f$\pmb{K}_s = 0\f$ and a diffuse vector \f$\pmb{K}_d \neq 0\f$
   *  with at least one non-null component.
   * @return True if the material is Lambert (non-specular), False otherwise.
   */
  bool isLambert() const;
  /**
   * @brief A material is said to be direction independent when it does not
   *  account for either specular or diffuse lighting modeling.
   * @return True if the material is uniform (null specular and diffuse
   *  vectors), False otherwise
   */
  bool isDirectionIndependent() const;
};
