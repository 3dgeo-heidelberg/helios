#pragma once

#include <RandomnessGenerator.h>
#include <noise/NoiseSource.h>
#include <string>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Abstract class for random noise handling
 *
 * @tparam RealType Type for the noise.
 */
template<typename RealType>
class RandomNoiseSource : public NoiseSource<RealType>
{
public:
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief RandomnessGenerator to be used to generate random noise
   */
  RandomnessGenerator<RealType> rg;

public:
  // ***  CONSTRUCTION  *** //
  // ********************** //
  /**
   * @brief Create a RandomNoiseSource using received RandomnessGenerator
   * @param rg RandomnessGenerator used to generate noise
   */
  explicit RandomNoiseSource(RandomnessGenerator<RealType> const& rg)
    : rg(rg)
  {
  }
  /**
   * @brief Create a RandomNoiseSource using received seed
   * @param seed Seed to build the RandomnessGenerator
   * @see RandomnessGenerator
   */
  explicit RandomNoiseSource(std::string const& seed)
    : rg(RandomnessGenerator<RealType>(seed))
  {
  }
  /**
   * @brief Create a RandomNoiseSource using default RandomnessGenerator
   */
  explicit RandomNoiseSource()
    : rg(RandomnessGenerator<RealType>())
  {
  }

  // ***  NOISE CONFIGURATION FUNCTIONS  *** //
  // *************************************** //
  /**
   * @brief Obtain the random noise type
   * @return Noise type
   */
  virtual std::string getRandomNoiseType() = 0;

  // ***  STREAM OPERATORS  *** //
  // ************************** //
  /**
   * @brief Output stream behavior
   */
  template<typename _RealType>
  friend std::ostream& operator<<(std::ostream& out,
                                  RandomNoiseSource<_RealType>& ns);
};

// ***  STREAM OPERATORS  *** //
// ************************** //
template<typename RealType>
std::ostream&
operator<<(std::ostream& out, RandomNoiseSource<RealType>& ns)
{
  out << static_cast<NoiseSource<RealType>&>(ns);
  out << "\tRandomNoiseSource:\n"
      << "\t\trandomNoiseType = " << ns.getRandomNoiseType() << "\n";
  return out;
}
