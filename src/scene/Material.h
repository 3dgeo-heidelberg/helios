#pragma once

#include <string>
#include <cmath>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

/**
 * @brief Class representing a material specification
 */
class Material {
    // ***  SERIALIZATION  *** //
    // *********************** //
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive &ar, const unsigned int version) {
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
    Material(const Material &sp);
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
    void findNonNullComponents(bool &nonNullKs, bool &nonNullKd) const;
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

#ifdef PYTHON_BINDING
	/**
	 * @brief PyHelios getter to obtain first ambient component
	 * @return First ambient component
	 */
	inline float getKa0() {return ka[0];}
	/**
	 * @brief PyHelios setter to set first ambient component
	 * @param ka0 New first ambient component
	 */
	inline void setKa0(double ka0) {ka[0] = ka0;}
	/**
	 * @brief PyHelios getter to obtain second ambient component
	 * @return Second ambient component
	 */
    inline float getKa1() {return ka[1];}
    /**
     * @brief PyHelios setter to set second ambient component
     * @param ka1 New second ambient component
     */
    inline void setKa1(double ka1) {ka[1] = ka1;}
    /**
     * @brief PyHelios getter to obtain third ambient component
     * @return Third ambient component
     */
    inline float getKa2() {return ka[2];}
    /**
     * @brief PyHelios setter to obtain third ambient component
     * @param ka2 New third ambient component
     */
    inline void setKa2(double ka2) {ka[2] = ka2;}
    /**
     * @brief PyHelios getter to obtain fourth ambient component
     * @return Fourth ambient component
     */
    inline float getKa3() {return ka[3];}
    /**
     * @brief PyHelios setter to set fourth ambient component
     * @param ka3 New fourth ambient component
     */
    inline void setKa3(double ka3) {ka[3] = ka3;}
    /**
     * @brief PyHelios getter to obtain first diffuse component
     * @return First diffuse component
     */
    inline float getKd0() {return kd[0];}
    /**
     * @brief PyHelios setter to set first diffuse component
     * @param kd0 New first diffuse component
     */
    inline void setKd0(double kd0) {kd[0] = kd0;}
    /**
     * @brief PyHelios getter to obtain second diffuse component
     * @return Second diffuse component
     */
    inline float getKd1() {return kd[1];}
    /**
     * @brief PyHelios setter to obtain second diffuse component
     * @param kd1 Second diffuse component
     */
    inline void setKd1(double kd1) {kd[1] = kd1;}
    /**
     * @brief PyHelios getter to obtain third diffuse component
     * @return Third diffuse component
     */
    inline float getKd2() {return kd[2];}
    /**
     * @brief PyHelios setter to set third diffuse component
     * @param kd2 New third diffuse component
     */
    inline void setKd2(double kd2) {kd[2] = kd2;}
    /**
     * @brief PyHelios getter to obtain fourth diffuse component
     * @return Fourth diffuse component
     */
    inline float getKd3() {return kd[3];}
    /**
     * @brief PyHelios setter to set fourth diffuse component
     * @param kd3 New fourth diffuse component
     */
    inline void setKd3(double kd3) {kd[3] = kd3;}
    /**
     * @brief PyHelios getter to obtain first specular component
     * @return First specular component
     */
    inline float getKs0() {return ks[0];}
    /**
     * @brief PyHelios setter to set first specular component
     * @param ks0 New first specular component
     */
    inline void setKs0(double ks0) {ks[0] = ks0;}
    /**
     * @brief PyHelios getter to obtain second specular component
     * @return Second specular component
     */
    inline float getKs1() {return ks[1];}
    /**
     * @brief PyHelios setter to set second specular component
     * @param ks1 New second specular component
     */
    inline void setKs1(double ks1) {ks[1] = ks1;}
    /**
     * @brief PyHelios getter to get third specular component
     * @return Third specular component
     */
    inline float getKs2() {return ks[2];}
    /**
     * @brief PyHelios setter to set third specular component
     * @param ks2 New third specular component
     */
    inline void setKs2(double ks2) {ks[2] = ks2;}
    /**
     * @brief PyHelios getter to get fourth specular component
     * @return Fourth specular component
     */
    inline float getKs3() {return ks[3];}
    /**
     * @brief PyHelios setter to set fourth specular component
     * @param ks3 New fourth specular component
     */
    inline void setKs3(double ks3) {ks[3] = ks3;}
#endif

};