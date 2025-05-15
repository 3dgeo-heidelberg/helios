#pragma once

#include <cstdlib>
#define _USE_MATH_DEFINES
#include <cmath>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class to handle discrete time. It supports transforming from
 *  continuous time to discrete time and also transforming from discrete time
 *  to continuous time. It also supports cyclic (normalized period) and
 *  periodic (for any given period scale, which matches normalized when it is
 *  \f$1\f$) transforms
 */
class DiscreteTime
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The frequency \f$F\f$ defining the discrete time. It is, how many
   *  iterations per second.
   * @see DiscreteTime::freq
   * @see DiscreteTime::period
   */
  std::size_t frequency;
  /**
   * @brief Decimal representation of the frequency \f$F\f$
   * @see DiscreteTime::frequency
   * @see DiscreteTime::period
   */
  double freq;
  /**
   * @brief Decimal representation of the period \f$\frac{1}{F} = F^{-1}\f$
   *
   * It can be understood as how many continuous time does each step take
   * @see DiscreteTime::frequency
   * @see DiscreteTime::freq
   * @see DiscreteTime::periodScale
   */
  double period;
  /**
   * @brief The period scale \f$p\f$ defining the period length. If it is
   *  1.0, then it means periodic and cyclic transforms match. Otherwise,
   *  cyclic transforms work in \f$[0, 1)\f$ continuous interval and
   *  \f$[0, F)\f$ discrete interval. While periodic transforms would work in
   *  \f$[0, p)\f$ continuous interval and \f$[0, pF)\f$ discrete interval.
   * @see DiscreteTime::toPeriodicDiscrete
   * @see DiscreteTime::toPeriodicContinuous
   * @see DiscreteTime::freqScale
   * @see DiscreteTime::period
   */
  double periodScale;
  /**
   * @brief Simply the inverse of period scale \f$\frac{1}{p} = p^{-1}\f$
   * @see DiscreteTime::periodScale
   */
  double freqScale;
  /**
   * @brief How many iterations applying period scale \f$p\f$. It is,
   *  \f$pF\f$
   * @see DiscreteTime::periodScale
   * @see DiscreteTime::scaledFreq
   */
  std::size_t scaledFrequency;
  /**
   * @brief Decimal representation of the scaled frequency \f$pF\f$
   * @see DiscreteTime::scaledFrequency
   */
  double scaledFreq;
  /**
   * @brief The scaled period \f$\frac{1}{pF^2}
   */
  double scaledPeriod;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief The main constructor for DiscreteTime
   * @see DiscreteTime:frequency
   */
  DiscreteTime(std::size_t const frequency, double const periodScale = 1.0);
  virtual ~DiscreteTime() = default;

  // ***  TRANSFORMS  *** //
  // ******************** //
  /**
   * @brief Transform given continuous time to its closest representation
   *  in discrete time
   * @param time Continuous time to be transformed to discrete time
   * @return Closest representation of given continuous time as discrete time
   * @see DiscreteTime::toContinuous
   * @see DiscreteTime::toCyclicDiscrete
   * @see DiscreteTime::toPeriodicDiscrete
   */
  virtual std::size_t toDiscrete(double const time) const;
  /**
   * @brief Like DiscreteTime::toDiscrete but forcing the step to be always
   *  inside \f$[0, F)\f$ interval. If given continuous time is
   *  \f$\geq1.0\f$, then it is automatically handled to find its
   *  corresponding step with respect to \f$[0, 1)\f$ continuous time domain.
   * @see DiscreteTime::toDiscrete
   * @see DiscreteTime::toPeriodicDiscrete
   */
  virtual std::size_t toCyclicDiscrete(double const time) const;
  /**
   * @brief Like DiscreteTime::toCyclicDiscrete but taking period scale into
   *  account
   * @see DiscreteTime::toCyclicDiscrete
   * @see DiscreteTime::toDiscrete
   * @see DiscreteTime::periodScale
   */
  virtual std::size_t toPeriodicDiscrete(double const time) const;
  /**
   * @brief Transform given discrete time to its closest representation
   *  in continuous time
   * @param step Discrete time step to be transformed to continuous time
   * @return Closets representation of given discrete time as continuous time
   * @see DiscreteTime::toDiscrete
   * @see DiscreteTime::toCyclicContinuous
   */
  virtual double toContinuous(std::size_t const step) const;
  /**
   * @brief Like DiscreteTime::toContinuous but forcing the time to be always
   *  inside \f$[0, 1)\f$ interval. If given discrete time is \f$\geq F\f$,
   *  then it is automatically handled to find its corresponding continuous
   *  time with respect to \f$[0, F)\f$ discrete time domain.
   * @see DiscreteTime::toContinuous
   */
  virtual double toCyclicContinuous(std::size_t const step) const;
  /**
   * @brief Like DiscreteTime::toCyclicContinuous but taking period scale
   *  into account
   * @see DiscreteTime::toCyclicContinuous
   * @see DiscreteTime::toContinuous
   * @see DiscreteTime::periodScale
   */
  virtual double toPeriodicContinuous(std::size_t const step) const;

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Obtain the frequency defining the discrete time
   * @return The frequency defining the discrete time
   * @see DiscreteTime::frequency
   * @see DiscreteTime::setFrequency
   */
  inline std::size_t getFrequency() const { return frequency; }
  /**
   * @brief Set the frequency defining the discrete time
   * @param frequency New frequency to define the discrete time
   * @see DiscreteTime::frequency
   * @see DiscreteTime::getFrequency
   */
  inline void setFrequency(std::size_t const frequency)
  {
    this->frequency = frequency;
    this->freq = (double)frequency;
    this->period = 1.0 / (this->freq);
    setPeriodScale(getPeriodScale()); // Update scaled frequencies
  }
  /**
   * @brief Obtain the period defining the discrete time
   * @return The period defining the discrete time
   */
  inline double getPeriod() const { return period; }
  /**
   * @brief Obtain the period scale defining the periodic discrete time
   * @return The period scale defining the periodic discrete time
   * @see DiscreteTime::periodScale
   */
  inline double getPeriodScale() const { return periodScale; }
  /**
   * @brief Set the new period scale defining the periodic discrete time
   * @param periodScale The new period scale
   * @see DiscreteTime::periodScale
   */
  inline void setPeriodScale(double const periodScale)
  {
    this->periodScale = periodScale;
    this->freqScale = 1.0 / periodScale;
    this->scaledFreq = periodScale * freq;
    this->scaledFrequency = (size_t)std::ceil(this->scaledFreq);
    this->scaledPeriod = period / scaledFreq;
  }
};
