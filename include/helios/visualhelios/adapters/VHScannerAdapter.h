#ifdef PCL_BINDING

#pragma once

#include <helios/scanner/Scanner.h>
#include <helios/sim/comps/Survey.h>

#include <glm/glm.hpp>

namespace visualhelios {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Abstract class defining core mechanism to adapt scanners to the
 *  visual Helios context based on PCL and VTK libraries
 */
class VHScannerAdapter
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The adapted scanner
   * @see Scanner
   */
  Scanner& scanner;
  /**
   * @brief The survey to which the adapted scanner belongs to
   * @see Survey
   */
  Survey& survey;
  /**
   * @brief The index of the current scanning leg
   * @see Simulation::mCurrentLegIndex
   * @see Leg
   */
  unsigned int currentLegIndex = 0;
  /**
   * @brief The coordinates of the origin for the current ray
   */
  glm::dvec3 origin;
  /**
   * @brief The director vector of the current ray
   */
  glm::dvec3 dir;
  /**
   * @brief The radius of the sphere representing the origin of the ray
   */
  double originRadius = 1.0;
  /**
   * @brief The RGB color of the sphere representing the origin of the ray
   */
  double originColor[3] = { 0.6, 0.1, 0.1 };
  /**
   * @brief The length of the ray. It specifies the magnitude of the vector
   *  representing the ray
   */
  double rayLength = 1.0;
  /**
   * @brief The color of the line representing the ray
   */
  double rayColor[3] = { 0.9, 0.1, 0.15 };
  /**
   * @brief The color of the line representing the ray when it is not
   *  returning an echo
   */
  double nonReturningRayColor[3] = { 0.3, 0.2, 0.9 };

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for the visual Helios scanner adapter
   * @param scanner The scanner object to be adapter for visual Helios
   */
  VHScannerAdapter(Scanner& scanner, Survey& survey)
    : scanner(scanner)
    , survey(survey)
  {
  }
  virtual ~VHScannerAdapter() = default;

  // ***  SIMULATION  *** //
  // ******************** //
  /**
   * @brief Start the scanner
   */
  void start();
  /**
   * @brief Compute the next step for the scanner, with respect to the
   *  current one
   */
  void nextStep();

  /**
   * @brief Start specified leg
   *
   * This method is a lighter version of the SurveyPlayback::startLeg method
   *  which only considers those features which are relevant for ray casting
   *  visualization.
   *
   * @see SurveyPlayback::startLeg
   */
  void startLeg(unsigned int const legIndex, bool const manual);
  /**
   * @brief Perform stop and turn operation to advance to next leg
   * @see SurveyPlayback::stopAndTurn
   */
  void stopAndTurn(unsigned int legIndex, shared_ptr<Leg> leg);

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain the scanner object
   *
   * <b><span style="color: red;">WARNING</span></b> this getter returns the
   *  scanner object reference allowing modifications.
   *  Use with caution
   *
   * @return Scanner object
   */
  inline Scanner& getScanner() { return scanner; }
  /**
   * @brief Return a constant reference to the origin of the current ray
   * @return Constant reference to the origin of the current ray
   */
  inline glm::dvec3 const& getRayOrigin() const { return origin; }
  /**
   * @brief Return a constant reference to the director vector of the current
   *  ray
   * @return Constant reference to the director vector of the current ray
   */
  inline glm::dvec3 const& getRayDir() const { return dir; }
  /**
   * @brief Obtain the ray's origin radius
   * @return The ray's origin radius
   * @see VHScannerAdapter::originRadius
   */
  inline double getOriginRadius() const { return originRadius; }
  /**
   * @brief Set the ray's origin radius
   * @param originRadius The new radius for the visualization of ray's origin
   * @see VHScannerAdapter::originRadius
   */
  inline void setOriginRadius(double const originRadius)
  {
    this->originRadius = originRadius;
  }
  /**
   * @brief Obtain the ray's origin color
   * @return The ray's origin color
   * @see VHScannerAdapter::originColor
   */
  inline double const* getOriginColor() const { return originColor; }
  /**
   * @brief Obtain the ray's origin red color component
   * @return They ray's origin red color component
   * @see VHScannerAdapter::originColor
   */
  inline double getOriginColorRed() const { return originColor[0]; }
  /**
   * @brief Obtain the ray's origin blue color component
   * @return They ray's origin blue color component
   * @see VHScannerAdapter::originColor
   */
  inline double getOriginColorBlue() const { return originColor[1]; }
  /**
   * @brief Obtain the ray's origin green color component
   * @return They ray's origin green color component
   * @see VHScannerAdapter::originColor
   */
  inline double getOriginColorGreen() const { return originColor[2]; }
  /**
   * @brief Set the ray's origin color
   * @param rgb Pointer to an array with at least 3 components such that
   *  the first one is the red color, the second one is the green color and
   *  the third one is the blue color.
   * @see VHScannerAdapter::originColor
   */
  inline void setOriginColor(double const* rgb)
  {
    setOriginColor(rgb[0], rgb[1], rgb[2]);
  }
  /**
   * @brief Set the ray's origin color
   * @param r The red component for the ray's origin color
   * @param g The green component for the ray's origin color
   * @param b The blue component for the ray's origin color
   * @see VHScannerAdapter::originColor
   */
  inline void setOriginColor(double const r, double const g, double const b)
  {
    originColor[0] = r;
    originColor[1] = g;
    originColor[2] = b;
  }
  /**
   * @brief Obtain the ray's length
   * @return The ray's length
   * @see VHScannerAdapter::rayLength
   */
  inline double getRayLength() const { return rayLength; }
  /**
   * @brief Set the ray's length
   * @param rayLength The new length for the visualization of rays
   * @see VHScannerAdapter::rayLength
   */
  inline void setRayLength(double const rayLength)
  {
    this->rayLength = rayLength;
  }
  /**
   * @brief Obtain the ray's color
   * @return The ray's color
   * @see VHScannerAdapter::rayColor
   */
  inline double const* getRayColor() const { return rayColor; }
  /**
   * @brief Obtain the ray's red color component
   * @return The ray's red color component
   * @see VHScannerAdapter::rayColor
   */
  inline double getRayColorRed() const { return rayColor[0]; }
  /**
   * @brief Obtain the ray's green color component
   * @return The ray's green color component
   * @see VHScannerAdapter::rayColor
   */
  inline double getRayColorGreen() const { return rayColor[1]; }
  /**
   * @brief Obtain the ray's blue color component
   * @return The ray's blue color component
   * @see VHScannerAdapter::rayColor
   */
  inline double getRayColorBlue() const { return rayColor[2]; }
  /**
   * @brief Set the ray's color
   * @param rgb Pointer to an array with at least 3 components such that
   *  the first one is the red color, the second one is the green color
   *  and the third one is the blue color.
   * @see VHScannerAdapter::rayColor
   */
  inline void setRayColor(double const* rgb)
  {
    setRayColor(rgb[0], rgb[1], rgb[2]);
  }
  /**
   * @brief Set the ray's color
   * @param r The red component for the ray's color
   * @param g The green component for the ray's color
   * @param b The blue component for the ray's color
   * @see VHScannerAdapter::rayColor
   */
  inline void setRayColor(double const r, double const g, double const b)
  {
    rayColor[0] = r;
    rayColor[1] = g;
    rayColor[2] = b;
  }
  /**
   * @brief Obtain the non returning ray's color
   * @return The non returning ray's color
   * @see VHScannerAdapter::nonReturningRayColor
   */
  inline double const* getNonReturningRayColor() const
  {
    return nonReturningRayColor;
  }
  /**
   * @brief Obtain the non returning ray's red color component
   * @return The non returning ray's red color component
   * @see VHScannerAdapter::nonReturningRayColor
   */
  inline double getNonReturningRayColorRed() const
  {
    return nonReturningRayColor[0];
  }
  /**
   * @brief Obtain the non returning ray's green color component
   * @return The non returning ray's green color component
   * @see VHScannerAdapter::nonReturningRayColor
   */
  inline double getNonReturningRayColorGreen() const
  {
    return nonReturningRayColor[1];
  }
  /**
   * @brief Obtain the non returning ray's blue color component
   * @return The non returning ray's blue color component
   * @see VHScannerAdapter::nonReturningRayColor
   */
  inline double getNonReturningRayColorBlue() const
  {
    return nonReturningRayColor[2];
  }
  /**
   * @brief Set the non returning ray's color
   * @param rgb Pointer to an array with at least 3 components such that
   *  the first one is the red color, the second one is the green color
   *  and the third one is the blue color.
   * @see VHScannerAdapter::nonReturningRayColor
   */
  inline void setNonReturningRayColor(double const* rgb)
  {
    setNonReturningRayColor(rgb[0], rgb[1], rgb[2]);
  }
  /**
   * @brief Set the non returning ray's color
   * @param r The red
   * @param r The red component for the ray's color
   * @param g The green component for the ray's color
   * @param b The blue component for the ray's color
   * @see VHScannerAdapter::nonReturningRayColor
   */
  inline void setNonReturningRayColor(double const r,
                                      double const g,
                                      double const b)
  {
    nonReturningRayColor[0] = r;
    nonReturningRayColor[1] = g;
    nonReturningRayColor[2] = b;
  }
  /**
   * @brief Obtain the scanner's pulse frequency, in Hertz
   * @return The scanner's pulse frequency, in Hertz
   * @see Scanner::getPulseFreq_Hz
   */
  inline int getPulseFreq_Hz() const { return scanner.getPulseFreq_Hz(); }
  /**
   * @brief Set the scanner's pulse frequency, in Hertz
   * @param pulseFreq_Hz The new pulse frequency (in Hertz) for the scanner
   * @see Scanner::setPulseFreq_Hz
   */
  inline void setPulseFreq_Hz(int const pulseFreq_Hz)
  {
    scanner.setPulseFreq_Hz(pulseFreq_Hz);
  }
  /**
   * @brief Obtain the ray color depending on if the ray is expected to
   *  return an echo or not.
   *
   * NOTE that it is not the ray color depending on if the ray returns an
   *  encho or not, but if it is EXPECTED to so or not. This means that the
   *  ray intersection check is not computed. Instead, only the angular
   *  domain restrictions are applied to check whether the ray is expected
   *  to return an echo or not.
   *
   * @return The ray color depending on if the ray is expected to return an
   *  echo or not
   */
  inline double const* getCurrentRayColor() const
  {
    if (scanner.getBeamDeflector()->lastPulseLeftDevice()) {
      return getRayColor();
    }
    return getNonReturningRayColor();
  }
};

}

#endif
