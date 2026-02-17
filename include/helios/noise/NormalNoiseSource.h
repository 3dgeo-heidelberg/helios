#pragma once

#include <helios/noise/RandomNoiseSource.h>
#include <string>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class for normal noise handling
 *
 * @tparam RealType Type for the noise. Should be a decimal type, for instance
 * float or double.
 */
template<typename RealType>
class NormalNoiseSource : public RandomNoiseSource<RealType>
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The mean for the normal noise computation
   */
  RealType normalNoiseMean = 0.0;
  /**
   * @brief the standard deviation for hte normal noise computation
   */
  RealType normalNoiseStdev = 1.0;

public:
  // *** CONSTRUCTION  *** //
  // ********************* //
  /**
   * @brief Create a NormalNoiseSource using received RandomnessGenerator
   * @param rg RandomnessGenerator used to generate normal noise
   * @see normalNoiseMean
   * @see normalNoiseStdev
   */
  explicit NormalNoiseSource(RandomnessGenerator<RealType> const& rg,
                             RealType normalNoiseMean = 0.0,
                             RealType normalNoiseStdev = 1.0)
    : RandomNoiseSource<RealType>(rg)
    , normalNoiseMean(normalNoiseMean)
    , normalNoiseStdev(normalNoiseStdev)
  {
    this->rg.computeNormalDistribution(normalNoiseMean, normalNoiseStdev);
    this->build();
  }
  /**
   * @brief Create a NormalNoiseSource using received seed
   * @param seed Seed to build the normal noise generator
   * @see normalNoiseMean
   * @see normalNoiseStdev
   */
  explicit NormalNoiseSource(std::string const& seed,
                             RealType normalNoiseMean = 0.0,
                             RealType normalNoiseStdev = 1.0)
    : RandomNoiseSource<RealType>(seed)
    , normalNoiseMean(normalNoiseMean)
    , normalNoiseStdev(normalNoiseStdev)
  {
    this->rg.computeNormalDistribution(normalNoiseMean, normalNoiseStdev);
    this->build();
  }
  /**
   * @brief Create default NormalNoiseSource
   * @see normalNoiseMean
   * @see normalNoiseStdev
   */
  explicit NormalNoiseSource(RealType normalNoiseMean = 0.0,
                             RealType normalNoiseStdev = 1.0)
    : RandomNoiseSource<RealType>()
    , normalNoiseMean(normalNoiseMean)
    , normalNoiseStdev(normalNoiseStdev)
  {
    this->rg.computeNormalDistribution(normalNoiseMean, normalNoiseStdev);
    this->build();
  }

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @brief Obtain the current mean for normal noise generation
   * @return Current mean for normal noise generation
   */
  RealType getMean() { return this->normalNoiseMean; }
  /**
   * @brief Set the current mean for normal noise generation
   *
   * NOTICE this will update the normal random distribution used to generate
   * noise. USE WITH CAUTION
   * If updating both mean and standard deviation at the same time is
   * desired, it is strongly recommended to use the configureNormalNoise
   * function directly instead
   *
   * @param mean New mean for normal noise generation
   * @return The NormalNoiseSource reference in a fluent programming fashion
   * @see configureNormalNoise
   */
  NormalNoiseSource& setMean(RealType mean)
  {
    return this->configureNormalNoise(mean, this->normalNoiseStdev);
  }
  /**
   * @brief Obtain the current standard deviation for normal noise generation
   * @return Current standard deviation for normal noise generation
   */
  RealType getStdev() { return this->normalNoiseStdev; }
  /**
   * @brief Set the current standard deviation for normal noise generation
   *
   * NOTICE this will update the normal random distribution used to generate
   * noise. USE WITH CAUTION
   * If updating both mean and standard deviation at the same time is
   * desired, it is strongly recommended to use the configureNormalNoise
   * function directly instead
   *
   * @param stdev New standard deviation for normal noise generation
   * @return The NormalNoiseSource reference in fluent programming fashion
   * @see configureNormalNoise
   */
  NormalNoiseSource& setStdev(RealType stdev)
  {
    return this->configureNormalNoise(this->normalNoiseMean, stdev);
  }

  // ***  NOISE CONFIGURATION FUNCTIONS  *** //
  // *************************************** //
  /**
   * @brief Configure normal noise
   * @param mean The mean for the normal noise
   * @param stdev The standard deviation for the normal noise
   * @return The NormalNoiseSource reference in a fluent programming fashion
   */
  NormalNoiseSource& configureNormalNoise(RealType mean, RealType stdev);
  /**
   * @see RandomNoiseSource::getRandomNoiseType()
   */
  std::string getRandomNoiseType() override { return "NORMAL"; }

  // ***  NOISE OBTAINMENT FUNCTIONS  *** //
  // ************************************ //
  /**
   * @brief Compute next normal nosie value from current random normal
   * distribution
   * @return Normal noise value
   */
  RealType noiseFunction() override
  {
    return this->rg.normalDistributionNext();
  }

  // ***  STREAM OPERATORS  *** //
  // ************************** //
  /**
   * @brief Output stream behavior
   */
  template<typename _RealType>
  friend std::ostream& operator<<(std::ostream& out,
                                  NormalNoiseSource<_RealType>& ns);
};

// ***   CLASS  IMPLEMENTATION   *** //
// ********************************* //
template<typename RealType>
NormalNoiseSource<RealType>&
NormalNoiseSource<RealType>::configureNormalNoise(RealType mean, RealType stdev)
{
  this->normalNoiseMean = mean;
  this->normalNoiseStdev = stdev;
  this->rg.computeNormalDistribution(mean, stdev);
  return *this;
}

// ***  STREAM OPERATORS  *** //
// ************************** //
template<typename RealType>
std::ostream&
operator<<(std::ostream& out, NormalNoiseSource<RealType>& ns)
{
  out << static_cast<RandomNoiseSource<RealType>&>(ns);
  out << "\t\tNormalNoiseSource:\n"
      << "\t\t\tnormalNoiseMean = " << ns.getMean() << "\n"
      << "\t\t\tnormalNoiseStdev = " << ns.getStdev() << "\n";
  return out;
}
