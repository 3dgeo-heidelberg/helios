#pragma once

#include <ostream>
#include <sstream>
#include <string>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class to handle a noise source
 *
 * @tparam RealType Type of the generated noise.
 * For instance double or float.
 */
template<typename RealType>
class NoiseSource
{
public:
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief All noise values which are less than clipMin will be clipped
   * to clipMin if clipping is enabled
   *
   * @see clipEnabled
   */
  RealType clipMin = 0.0;
  /**
   * @brief All noise values which are greater than clipMax will be clipped
   * to clipMaxx if clipping is enabled
   */
  RealType clipMax = 1.0;
  /**
   * @brief True when clipping is enabled, False otherwise.
   */
  bool clipEnabled = false;
  /**
   * @brief Specify the how many times a fixed value can be used before
   * being renewed.
   *
   * <br/><br/>
   *
   * NOTICE a value of 0 means the fixed value will not be renewed, unless
   * manually specified through fixedRenew function invocation.
   *
   * <br/><br/>
   *
   * A value of 1 means the fixed value will be renewed between each pair
   * of consecutive uses, hence it behaves like if it was not a fixed value
   *
   * <br/><br/>
   *
   * Having fixed enabled (fixedLifespan != 1) means a noise value will be
   * computed one time and always that a next noise value is requested the
   * previously computed fixed value will be returned, as long as its
   * lifespan has not expired yet
   *
   * <br/><br/>
   *
   * <b>WARNING</b> fixed value only is considered when obtaining noise
   * through next function.
   * Specific noise functions do not support fixed values.
   */
  unsigned long fixedLifespan = 1L;
  /**
   * @brief How many remaining uses the fixed value has
   *
   * @see fixedLifespan
   */
  unsigned long fixedRemainingUses = 0L;
  /**
   * @brief The fixed value
   *
   * @see fixedLifespan
   */
  RealType fixedValue = 0;

public:
  // ***  CONSTRUCTION  *** //
  // ********************** //
  /**
   * @brief Common behavior for all NoiseSource constructors
   */
  void build() { this->fixedValue = noiseFunction(); }

  // ***  CLIPPING FUNCTIONS  *** //
  // **************************** //
  /**
   * @brief Obtain the clip min value
   * @return Clip min value
   * @see clipMin
   */
  inline double getClipMin() { return clipMin; }
  /**
   * @brief Set the clip min value
   * @param clipMin New clip min value
   * @return The NoiseSource reference in a fluent programming fashion
   * @see clipMin
   */
  inline NoiseSource& setClipMin(RealType clipMin)
  {
    this->clipMin = clipMin;
    return *this;
  }
  /**
   * @brief Obtain the clip max value
   * @return Clip max value
   * @see clipMax
   */
  inline double getClipMax() { return clipMax; }
  /**
   * @brief Set the clip max value
   * @param clipMax New clip max value
   * @return The NoiseSource reference in a fluent programming fashion
   * @see clipMax
   */
  inline NoiseSource& setClipMax(RealType clipMax)
  {
    this->clipMax = clipMax;
    return *this;
  }
  /**
   * @brief Check if clipping is enabled or not
   * @return True if clipping is enabled, false otherwise
   * @see clipEnabled
   */
  inline bool isClipEnabled() { return clipEnabled; }
  /**
   * @brief Enable clipping by setting it to true or disable it by setting
   * to false.
   * @param clipEnabled True to enable clipping, False to disable it
   * @return The NoiseSource reference in a fluent programming fashion
   * @see clipEnabled
   */
  inline NoiseSource& setClipEnabled(bool clipEnabled)
  {
    this->clipEnabled = clipEnabled;
    return *this;
  }

protected:
  /**
   * @brief If clipping is enabled, it will clip received value.
   * If clipping is not enabled, received value will not be clipped.
   * @return Clipped or not clipped received value, depending on if
   * clipping is enabled or not
   * @see clipEnabled
   */
  inline RealType clip(RealType v);

public:
  // ***  NOISE CONFIGURATION FUNCTIONS  *** //
  // *************************************** //
  /**
   * @brief Check if fixed value usage is enabled or not
   * @return True if fixed value usage is enabled, false otherwise
   */
  bool isFixedValueEnabled() { return this->fixedLifespan != 1L; }
  /**
   * @brief Obtain the fixed value lifespan
   * @return Fixed value lifespan
   * @see fixedLifespan
   */
  unsigned long getFixedLifespan() { return this->fixedLifespan; }
  /**
   * @brief Set the fixed value lifespan
   *
   * NOTICE this does not update fixed value remaining uses
   *
   * @param fixedLifespan The new lifespan for fixed values
   * @return The NoiseSource reference in a fluent programming fashion
   */
  NoiseSource& setFixedLifespan(unsigned long fixedLifespan)
  {
    this->fixedLifespan = fixedLifespan;
    return *this;
  }
  /**
   * @brief Obtain the remaining uses of current fixed value
   *
   * NOTICE lifespan might be eternal (0L), in which case remaining uses
   * count does not affect the behavior of the noise source
   *
   * @return Remaining uses for current fixed value
   * @see fixedRemainingUses
   */
  unsigned long getFixedValueRemainingUses()
  {
    return this->fixedRemainingUses;
  }
  /**
   * @brief Update remaining uses count for current fixed value
   * @param remainingUses New remaining uses count
   * @return The NoiseSource reference in a fluent programming fashion
   */
  NoiseSource& setFixedValueRemainingUses(unsigned long remainingUses)
  {
    fixedRemainingUses = remainingUses;
    return *this;
  }
  /**
   * @brief Forces a renewal of fixed value and its remaining uses
   * @return The NoiseSource in a fluent programming fashion
   * @see fixedLifespan
   */
  NoiseSource& fixedRenew();

  // ***  NOISE OBTAINMENT FUNCTIONS  *** //
  // ************************************ //
  /**
   * @brief Obtain the next default noise value
   *
   * NOTICE this function considers fixed values.
   *
   * @return Next noise value for default noise
   * @see defaultNoiseType
   * @see fixedLifespan
   */
  RealType next();

  /**
   * @brief Function which computes noise values
   * @return Computed noise value
   */
  virtual RealType noiseFunction() = 0;

  // ***  STREAM OPERATORS  *** //
  // ************************** //
  /**
   * @brief Output stream behavior
   */
  template<typename _RealType>
  friend std::ostream& operator<<(std::ostream& out,
                                  NoiseSource<_RealType> const& ns);
};

// ***   CLASS  IMPLEMENTATION   *** //
// ********************************* //

// ***  CLIPPING FUNCTIONS  *** //
// **************************** //
template<typename RealType>
RealType
NoiseSource<RealType>::clip(RealType v)
{
  if (isClipEnabled()) {
    if (v > getClipMax())
      return getClipMax();
    if (v < getClipMin())
      return getClipMin();
  }

  return v;
}

// ***  NOISE CONFIGURATION FUNCTIONS  *** //
// *************************************** //
template<typename RealType>
NoiseSource<RealType>&
NoiseSource<RealType>::fixedRenew()
{
  fixedValue = noiseFunction();
  fixedRemainingUses = fixedLifespan;
  return *this;
}

// ***  NOISE OBTAINMENT FUNCTIONS  *** //
// ************************************ //
template<typename RealType>
RealType
NoiseSource<RealType>::next()
{
  // Handle fixed values
  if (fixedLifespan != 1L) {
    if (fixedLifespan == 0L)
      return fixedValue;
    if (fixedRemainingUses > 0L) {
      fixedRemainingUses--;
      return fixedValue;
    }
  }

  // Obtain noise value
  fixedValue = clip(noiseFunction());

  // Update remaining uses if necessary
  if (fixedLifespan > 1L) {
    fixedRemainingUses = fixedLifespan - 1;
  }

  // Return noise value
  return fixedValue;
}

// ***  STREAM OPERATORS  *** //
// ************************** //
template<typename RealType>
std::ostream&
operator<<(std::ostream& out, NoiseSource<RealType> const& ns)
{
  std::stringstream ss;
  ss << "Noise source (" << &ns << "):\n"
     << "\tclipMin = " << ns.clipMin << "\n"
     << "\tclipMax = " << ns.clipMax << "\n"
     << "\tclipEnabled = " << ns.clipEnabled << "\n"
     << "\tfixedLifespan = " << ns.fixedLifespan << "\n"
     << "\tfixedRemainingUses = " << ns.fixedRemainingUses << "\n"
     << "\tfixedValue = " << ns.fixedValue << "\n";
  out << ss.str();
  return out;
}
