#include <DiscreteTime.h>

using std::size_t;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
DiscreteTime::DiscreteTime(std::size_t const frequency,
                           double const periodScale)
  : periodScale(periodScale)
{
  setFrequency(frequency);
  setPeriodScale(periodScale);
}

// ***  TRANSFORMS  *** //
// ******************** //
std::size_t
DiscreteTime::toDiscrete(double const time) const
{
  return (std::size_t)(time * freq);
}

std::size_t
DiscreteTime::toCyclicDiscrete(double const time) const
{
  return toDiscrete(time - std::floor(time));
}

std::size_t
DiscreteTime::toPeriodicDiscrete(double const time) const
{
  return ((time - std::floor(time * freqScale) * periodScale) * freqScale) *
         scaledFreq;
}

double
DiscreteTime::toContinuous(std::size_t const step) const
{
  return ((double)step) * period;
}

double
DiscreteTime::toCyclicContinuous(std::size_t const step) const
{
  if (step < frequency)
    return toContinuous(step);
  return toContinuous(step - (step / frequency) * frequency);
}

double
DiscreteTime::toPeriodicContinuous(std::size_t const step) const
{
  return ((double)(step - (step / scaledFrequency) * scaledFrequency)) * period;
}
