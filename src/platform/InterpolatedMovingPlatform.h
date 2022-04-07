#pragma once

#include <platform/MovingPlatform.h>
#include <platform/trajectory/TrajectoryFunction.h>
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
     * @see SimulationStepLopp
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
    std::shared_ptr<TrajectoryFunction> tf = nullptr;
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
     * @see InterpolatedMovingPlatform::InterpolationScope
     * @see InterpolatedMovingPlatform::stepLoop
     */
    InterpolatedMovingPlatform(
        SimulationStepLoop &stepLoop,
        TemporalDesignMatrix<double, double> const &tdm,
        DiffDesignMatrix<double, double> const &ddm,
        InterpolationScope scope
    );
    virtual ~InterpolatedMovingPlatform() = default;

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @see Platform::doSimStep
     */
    void doSimStep(int simFrequency_hz) override;

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
    inline std::shared_ptr<TrajectoryFunction> getTrajectoryFunction() const
        {return tf;}
    /**
     * @brief Set the TrajectoryFunction
     * @see InterpolatedMovingPlatform::tf
     */
    inline void setTrajectoryFunction(std::shared_ptr<TrajectoryFunction> tf)
        {this->tf = tf;}
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
};