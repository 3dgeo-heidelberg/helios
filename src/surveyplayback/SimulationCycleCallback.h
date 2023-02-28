#pragma once

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Abstract class to handle simulation cycle callbacks
 *
 * A Simulation cycle is considered to end after simFrequency iterations
 * has elapsed. Once it finish, the callback function is invoked.
 */
class SimulationCycleCallback{
public:
    // ***  CONSTRUCTOR / DESTRUCTOR  *** //
    // ********************************** //
    /**
     * @brief Simulation cycle callback default constructor
     */
    SimulationCycleCallback() {}

    virtual ~SimulationCycleCallback() {}

    // ***  F U N C T O R  *** //
    // *********************** //
    /**
     * @brief Callback functor which operates over vector of measurements
     * @param measurements Vector of measurements to operate over
     * @param trajectories Vector of trajectories to operate over
     * @see Measurement
     * @see Trajectory
     */
    virtual void operator() (
        std::vector<Measurement> &measurements,
        std::vector<Trajectory> &trajectories
    ) = 0;
};