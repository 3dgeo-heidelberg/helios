#pragma once

#include <noise/RandomNoiseSource.h>
#include <string>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class for uniform noise handling
 *
 * @tparam RealType Type for the noise. Should be a decimal type, for instance
 * float or double.
 */
template<typename RealType>
class UniformNoiseSource : public RandomNoiseSource<RealType>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The minimum value the uniform noise is configured to support
   */
  RealType uniformNoiseMin = 0.0;
  /**
   * @brief The maximum value the uniform noise is configured to support
   */
  RealType uniformNoiseMax = 1.0;

public:
  // *** CONSTRUCTION  *** //
  // ********************* //
  /**
   * @brief Create a UniformNoiseSource using received RandomnessGenerator
   * @param rg RandomnessGenerator used to generate normal noise
   * @see uniformNoiseMin
   * @see uniformNoiseMax
   */
  explicit UniformNoiseSource(RandomnessGenerator<RealType> const& rg,
                              RealType uniformNoiseMin = 0.0,
                              RealType uniformNoiseMax = 1.0)
    : RandomNoiseSource<RealType>(rg)
    , uniformNoiseMin(uniformNoiseMin)
    , uniformNoiseMax(uniformNoiseMax)
  {
    this->rg.computeUniformRealDistribution(uniformNoiseMin, uniformNoiseMax);
    this->build();
  }
  /**
   * @brief Create a UniformNoiseSource using received seed
   * @param seed Seed to build the normal noise generator
   * @see uniformNoiseMin
   * @see uniformNoiseMax
   */
  explicit UniformNoiseSource(std::string const& seed,
                              RealType uniformNoiseMin = 0.0,
                              RealType uniformNoiseMax = 1.0)
    : RandomNoiseSource<RealType>(seed)
    , uniformNoiseMin(uniformNoiseMin)
    , uniformNoiseMax(uniformNoiseMax)
  {
    this->rg.computeUniformRealDistribution(uniformNoiseMin, uniformNoiseMax);
    this->build();
  }
  /**
   * @brief Create default UniformNoiseSource
   * @see uniformNoiseMin
   * @see uniformNoiseMax
   */
  explicit UniformNoiseSource(RealType uniformNoiseMin = 0.0,
                              RealType uniformNoiseMax = 1.0)
    : RandomNoiseSource<RealType>()
    , uniformNoiseMin(uniformNoiseMin)
    , uniformNoiseMax(uniformNoiseMax)
  {
    this->rg.computeUniformRealDistribution(uniformNoiseMin, uniformNoiseMax);
    this->build();
  }

  // ***  GETTERS and SETTERS  *** //
  // ***************************** /
  /**
   * @brief Obtain the current minimum for uniform noise generation
   * @return Current minimum for uniform noise generation
   */
  RealType getMin() { return this->uniformNoiseMin; }
  /**
   * @brief Set the current minimum for uniform noise generation
   *
   * NOTICE this will update the uniform random distribution used to generate
   * noise. USE WITH CAUTION.
   * If updating both minimum and maximum at the same time is desired,
   * it is strongly recommended to use the configureUniformNoise function
   * directly instead
   *
   * @param min New minimum for uniform noise generation
   * @return The UniformNoiseSource reference in a fluent programming fashion
   * @see configureUniformNoise
   */
  UniformNoiseSource& setMin(RealType min)
  {
    return this->configureUniformNoise(min, this->uniformNoiseMax);
  }
  /**
   * @brief Obtain the current maximum for uniform noise generation
   * @return Current maximum for uniform noise generation
   */
  RealType getMax() { return this->uniformNoiseMax; }
  /**
   * @brief Set the current maximum for uniform noise generation
   *
   * NOTICE this will update the uniform random distribution used to generate
   * noise. USE WITH CAUTION.
   * If updating both minimum and maximum at the same time is desired,
   * it is strongly recommended to use the configureUniformNoise function
   * directly instead
   *
   * @param max New maximum for uniform noise generation
   * @return The UniformNoiseSource refenrece in a fluent programming fashion
   * @see configureUniformNoise
   */
  UniformNoiseSource& setMax(RealType max)
  {
    return this->configureUniformNoise(this->uniformNoiseMin, max);
  }

  // ***  NOISE CONFIGURATION FUNCTIONS  *** //
  // *************************************** //
  /**
   * @brief Configure uniform noise
   *
   * @param min Min possible value for uniform noise
   * @param max Max possible value for uniform noise
   * @return The UniformNoiseSource reference in a fluent programming fashion
   */
  UniformNoiseSource& configureUniformNoise(RealType min, RealType max);
  /**
   * @see RandomNoiseSource::getRandomNoiseType()
   */
  std::string getRandomNoiseType() override { return "UNIFORM"; }

  // ***  NOISE OBTAINMENT FUNCTIONS  *** //
  // ************************************ //
  /**
   * @brief Compute next uniform noise value from current random uniform
   * distribution
   * @return Uniform noise value
   */
  RealType noiseFunction() override
  {
    return this->rg.uniformRealDistributionNext();
  }

  // ***  STREAM OPERATORS  *** //
  // ************************** //
  /**
   * @brief Output stream behavior
   */
  template<typename _RealType>
  friend std::ostream& operator<<(std::ostream& out,
                                  UniformNoiseSource<_RealType>& ns);
};

// ***   CLASS  IMPLEMENTATION   *** //
// ********************************* //
template<typename RealType>
UniformNoiseSource<RealType>&
UniformNoiseSource<RealType>::configureUniformNoise(RealType min, RealType max)
{
  this->uniformNoiseMin = min;
  this->uniformNoiseMax = max;
  this->rg.computeUniformRealDistribution(min, max);
  return *this;
}

// ***  STREAM OPERATORS  *** //
// ************************** //
template<typename RealType>
std::ostream&
operator<<(std::ostream& out, UniformNoiseSource<RealType>& ns)
{
  out << static_cast<RandomNoiseSource<RealType>&>(ns);
  out << "\t\tUniformNoiseSource:\n"
      << "\t\t\tuniformNoiseMin = " << ns.getMin() << "\n"
      << "\t\t\tuniformNoiseMax = " << ns.getMax() << "\n";
  return out;
}
