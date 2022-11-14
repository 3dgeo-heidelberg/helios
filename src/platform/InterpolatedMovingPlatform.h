#pragma once

#include <platform/MovingPlatform.h>
#include <platform/trajectory/DesignTrajectoryFunction.h>
#include <fluxionum/TemporalDesignMatrix.h>
#include <fluxionum/DiffDesignMatrix.h>
#include <SimulationStepLoop.h>
#include <util/HeliosException.h>

#include <memory>
#include <functional>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class representing a MovingPlatform which position is defined by a
 *  function interpolated from a DesignMatrix
 *
 * @see MovingPlatform
 */
class InterpolatedMovingPlatform : public MovingPlatform{
public:
    /**
     * @brief Specify what is the interpolation scope. It is, which components
     *  are interpolated
     */
    enum class InterpolationScope{
        POSITION,
        ATTITUDE,
        POSITION_AND_ATTITUDE
    };

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Reference to the SimulationStepLoop defining the Simulation
     * @see SimulationStepLoop
     * @see Simulation
     * @see Simulation::stepLoop
     */
    SimulationStepLoop &stepLoop;
    /**
     * @brief Specify the scope of the interpolation
     * @see InterpolatedMovingPlatform::InterpolationScope
     */
    InterpolationScope scope;
    /**
     * @brief The trajectory function defining the platform's motion
     * @see TrajectoryFunction
     */
    std::shared_ptr<DesignTrajectoryFunction> tf = nullptr;
    /**
     * @brief The \f$m\f$ time frontiers \f$a_1, \ldots, a_m\f$ such that
     *  \f$\forall t,\, \exists i \ni t \in [a_i, a_{i+1})\f$.
     * @see InterpolatedMovingPlatform::frontierValues
     * @see InterpolatedMovingPlatform::frontierDerivatives
     */
    arma::Col<double> timeFrontiers;
    /**
     * @brief The \f$m\f$ vectors in \f$\mathbb{R}^{n}\f$ such that at the
     *  \f$i\f$-th frontier it is known that
     * \f$ \vec{y}(a_i) = \left(y_{i1}, \ldots, y_{in}\right) \f$
     * @see InterpolatedMovingPlatform::timeFrontiers
     * @see InterpolatedMovingPlatform::frontierDerivatives
     */
    arma::Mat<double> frontierValues;
    /**
     * @brief The \f$m\f$ vectors in \f$\mathbb{R}^{n}\f$ such that at the
     *  \f$i\f$-th frontier it is known that
     * \f$ \frac{d\vec{y}}{dt} = \left(
     *  \frac{dy_1}{dt}, \ldots, \frac{dy_n}{dt}
     * \right) \f$
     * @see InterpolatedMovingPlatform::timeFrontiers
     * @see InterpolatedMovingPlatform::frontierValues
     */
    arma::Mat<double> frontierDerivatives;
    /**
     * @brief Function which handles the update of components belonging to
     *  interpolation scope at each simulation step
     * @see InterpolatedMovingPlatform::doSimStep
     */
    std::function<void(double const t)> doStepUpdates;
    /**
     * @brief If true, the GPS time will be synchronized with the start time
     *  of the InterpolatedMovingPlatform. If false, nothing will be done.
     * @see InterpolatedMovingPlatform::startTime
     */
    bool syncGPSTime;
    /**
     * @brief The start time for the GPS time (in seconds).
     *
     * For the typical case, it is the smallest time value in the original time
     *  domain. It is, the smallest time value in the original data source
     *  before it was aligned to start at \f$t_0 = 0\f$.
     *
     * @see InterpolatedMovingPlatform::syncGPSTime
     */
    double startTime;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Build an InterpolatedMovingPlatform from given values and
     *  differentials. Note that the DiffDesignMatrix must define the
     *  derivatives for the same functions which values over time are described
     *  by the TemporalDesignMatrix
     * @param stepLoop Reference to the simulation step loop
     * @param tdm The matrix of values over time time
     * @param ddm The matrix of derivatives over time such that the \f$i\f$-th
     *  derivative associated with the \f$i\f$-th value at the \f$i\f$-th time
     * @param scope Specify the interpolation scope.
     * @param syncGPSTime Specify if GPS time must be synchronized (True) or
     *  not (false)
     * @param startTime The minimum time in the original time domain (the one
     *  used by the data source), it is given in seconds
     * @see InterpolatedMovingPlatform::InterpolationScope
     * @see InterpolatedMovingPlatform::stepLoop
     */
    InterpolatedMovingPlatform(
        SimulationStepLoop &stepLoop,
        TemporalDesignMatrix<double, double> const &tdm,
        DiffDesignMatrix<double, double> const &ddm,
        InterpolationScope scope,
        bool const syncGPSTime,
        double const startTime
    );
    virtual ~InterpolatedMovingPlatform() = default;

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @see Platform::doSimStep
     */
    void doSimStep(int simFrequency_hz) override;
    /**
     * @brief Check whether the interpolated moving platform has reached its
     *  waypoint or not.
     *
     * Let \f$t\f$ be the current time, and \f$a_m\f$ the last time frontier.
     *  Thus, the proposition "the waypoint of the interpolated platform has
     *  been reached" will be true if and only if \f$t \geq a_m\f$:
     *
     * \f[
     * \left\{\begin{array}{ll}
     *  \bot &,\; t < a_m \\
     *  \top &,\; t \geq a_m
     * \end{array}\right.
     * \f]
     *
	 * @see Platform::waypointReached
	 */
    bool waypointReached() override;
    /**
     * @brief Configures the iterative method of the trajectory function so
     *  the current iteration is considered to be at given time \f$t\f$.
     *
     * The step \f$h\f$ will be applied in consequence to reach \f$t+h\f$ for
     *  the new \f$t\f$
     */
    virtual void toTrajectoryTime(double const t);

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain the reference to the SimulationStepLoop
     * @see InterpolatedMovingPlatform::stepLoop
     */
    inline SimulationStepLoop & getStepLoop() const
        {return stepLoop;}
    /**
     * @brief Obtain the InterpolationScope
     * @see InterpolatedMovingPlatform::scope
     */
    inline InterpolationScope getScope() const
        {return scope;}
    /**
     * @brief Set the InterpolationScope
     * @see InterpolatedMovingPlatform::scope
     */
    inline void setScope(InterpolationScope const scope)
        {this->scope = scope;}
    /**
     * @brief Obtain the TrajectoryFunction
     * @see InterpolatedMovingPlatform::tf
     */
    inline std::shared_ptr<DesignTrajectoryFunction>
    getTrajectoryFunction() const
    {return tf;}
    /**
     * @brief Set the TrajectoryFunction
     * @see InterpolatedMovingPlatform::tf
     */
    inline void setTrajectoryFunction(
        std::shared_ptr<DesignTrajectoryFunction> tf
    ) {this->tf = tf;}
    /**
     * @brief Obtain the time frontiers
     * @see InterpolatedMovingPlatform::timeFrontiers
     */
    inline arma::Col<double> const & getTimeFrontiers() const
        {return timeFrontiers;}
    /**
     * @brief Set the time frontiers
     * @see InterpolatedMovingPlatform::timeFrontiers
     */
    inline void setTimeFrontiers(arma::Col<double> const &timeFrontiers)
        {this->timeFrontiers = timeFrontiers;}
    /**
     * @brief Obtain the values at the frontier points
     * @see InterpolatedMovingPlatform::frontierValues
     */
    inline arma::Mat<double> const & getFrontierValues() const
        {return frontierValues;}
    /**
     * @brief Set the values at the frontier points
     * @see InterpolatedMovingPlatform::frontierValues
     */
    inline void setFrontierValues(arma::Mat<double> const &frontierValues)
        {this->frontierValues = frontierValues;}
    /**
     * @brief Obtain the derivatives at the frontier points
     * @see InterpolatedMovingPlatform::frontierDerivatives
     */
    inline arma::Mat<double> const & getFrontierDerivatives() const
        {return frontierDerivatives;}
    /**
     * @brief Set the derivatives at the frontier points
     * @see InterpolatedMovingPlatform::frontierDerivatives
     */
    inline void setFrontierDerivatives(
        arma::Mat<double> const &frontierDerivatives
    ) {this->frontierDerivatives = frontierDerivatives;}
    /**
     * @brief Check whether the GPS time is requested to be synchronized with
     *  platform's start time (true) or not (false)
     * @see InterpolatedMovingPlatform::syncGPSTime
     */
    inline bool isSyncGPSTime() const {return syncGPSTime;}
    /**
     * @brief Either enable (true) or disable (false) the synchronization of
     *  GPS time with platform's start time
     * @see InterpolatedMovingPlatform::syncGPSTime
     */
    inline void setSyncGPSTime(bool const syncGPSTime)
    {this->syncGPSTime = syncGPSTime;}
    /**
     * @brief Obtain the start time
     * @see InterpolatedMovingPlatform::startTime
     */
    inline double getStartTime() const {return startTime;}
    /**
     * @brief Set the start time
     * @see InterpolatedMovingPlatform::startTime
     */
    inline void setStartTime(double const startTime)
    {this->startTime = startTime;}
    /**
     * @see Platform::isInterpolated
     */
    bool isInterpolated() const override {return true;}
};