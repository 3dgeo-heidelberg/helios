#include <helios/noise/RandomnessGenerator.h>

// ***  DEFAULT RANDOMNESS GENERATOR  *** //
// ************************************** //
bool DEFAULT_RG_MODIFIED_FLAG = false;
std::unique_ptr<RandomnessGenerator<double>> DEFAULT_RG(
  new RandomnessGenerator<double>());

void
setDefaultRandomnessGeneratorSeed(std::string const seed)
{
  if (seed == "") {
    DEFAULT_RG = std::unique_ptr<RandomnessGenerator<double>>(
      new RandomnessGenerator<double>());
  } else {
    DEFAULT_RG = std::unique_ptr<RandomnessGenerator<double>>(
      new RandomnessGenerator<double>(seed));
  }
  DEFAULT_RG_MODIFIED_FLAG = true;
}
