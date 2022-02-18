#pragma once

#include <LinearVoidStepLoop.h>
#include <DiscreteTime.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class extending LinearVoidStepLoop to support main simulation loop
 *
 * @see LinearVoidStepLoop
 * @see StepLoop
 * @see DiscreteTime
 */
class SimulationStepLoop : public LinearVoidStepLoop<>{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
	 * @brief The discrete time object to handle simulation frequency and time
	 */
    DiscreteTime dt;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Main constructor for simulation step loop
     * @param f The function to compute simulation steps
     */
    SimulationStepLoop(std::function<void(void)> f) :
        LinearVoidStepLoop(f),
        dt(1)
    {}
    virtual ~SimulationStepLoop() = default;

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @see DiscreteTime::getFrequency
     */
    inline std::size_t getFrequency() const {return dt.getFrequency();}
    /**
     * @see DiscreteTime::setFrequency
     */
    inline void setFrequency(std::size_t const frequency)
    {dt.setFrequency(frequency);}
    /**
     * @see DiscreteTime::getPeriod
     */
    inline double getPeriod() const {return dt.getPeriod();}
    /**
     * @see DiscreteTime::getPeriodScale
     */
    inline double getPeriodScale() const {return dt.getPeriodScale();}
    /**
     * @see DiscreteTime::setPeriodScale
     */
    inline void setPeriodScale(double const periodScale)
    {dt.setPeriodScale(periodScale);}
    /**
     * @brief Obtain the virtual time (simulation time) that elapsed to reach
     *  current step
     * @return Virtual time (simulation time) that elapsed to reach current
     *  step
     */
    inline double getCurrentTime() const
    {return dt.toContinuous(getCurrentStep());}

};