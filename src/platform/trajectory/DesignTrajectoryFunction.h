#pragma once

#include <platform/trajectory/TrajectoryFunction.h>
#include <fluxionum/ParametricClosestLesserSampleFunction.h>
#include <fluxionum/FixedParametricIterativeEulerMethod.h>

#include <armadillo>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class representing a trajectory function that comes from
 *  interpolating a TemporalDesignMatrix using its corresponding
 *  DiffDesignMatrix
 *
 * The basic implementation of the DesignTrajectoryFunction uses the
 *  FixedParametricIterativeEulerMethod to approximate the values
 *
 * @see TrajectoryFunction
 * @see fluxionum::TemporalDesignMatrix
 * @see fluxionum::DiffDesignMatrix
 * @see fluxionum::FixedParametricIterativeEulerMethod
 */
class DesignTrajectoryFunction : public TrajectoryFunction {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The function approximating the derivatives at a given
     *  time
     * \f[
     *  \frac{d\vec{x}}{dt} = \left(
     *      \frac{dx_1}{dt}, \ldots, \frac{dx_n}{dt}
     *  \right)
     * \f]
     */
    fluxionum::ParametricClosestLesserSampleFunction<double, double> pclsf;
    /**
     * @brief The function solving the Euler method to estimate the parametric
     *  function
     * \f[
     *  \vec{x}(t+h) = \vec{x}(t) + h \frac{d\vec{x}}{dt}
     * \f]
     */
    fluxionum::FixedParametricIterativeEulerMethod<double, double> fpiem;
    /**
     * @brief The last time value evaluated by this function. By default, it is
     *  0
     */
    double lastTime;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for DesignTrajectoryFunction
     * @see DesignTrajectoryFunction::timeFrontiers
     * @see DesignTrajectoryFunction::frontierValues
     * @see DesignTrajectoryFunction::frontierDerivatives
     */
    DesignTrajectoryFunction(
        arma::Col<double> const &timeFrontiers,
        arma::Mat<double> const &frontierValues,
        arma::Mat<double> const &frontierDerivatives
    ) :
        TrajectoryFunction(),
        pclsf(timeFrontiers, frontierDerivatives, 0),
        fpiem(
            pclsf,
            timeFrontiers(0),
            frontierValues.row(0).as_col(),
            timeFrontiers,
            frontierValues,
            0
        ),
        lastTime(0)
    {}
    virtual ~DesignTrajectoryFunction() = default;

    // ***  TRAJECTORY FUNCTION  *** //
    // ***************************** //
    /**
     * @brief Evaluate the function \f$f(t) = (x_1(t), \ldots, x_n(t))\f$
     * @param t The value defining a concrete time
     * @return The output as column vector \f$(x_1(t), \ldots, x_n(t))^T\f$
     */
    arma::Col<double> eval(double const &t) override{
        double const h = t-lastTime;
        lastTime = t;
        return fpiem(h);
    }

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain the parametric closest lesser sample function used to
     *  compute the design trajectory function
     * @return The parametric closest lesser sample function used to compute
     *  the design trajectory function
     * @see DesignTrajectoryFunction::pclsf
     * @see fluxionum::ParametricClosestLesserSampleFunction
     */
    fluxionum::ParametricClosestLesserSampleFunction<double, double> &
    getPclsf() {return pclsf;}
    /**
     * @brief Obtain the fixed parametric iterative Euler method used to
     *  compute the design trajectory function
     * @return The fixed parametric iterative Euler method used to compute the
     *  design trajectory function
     * @see DesignTrajectoryFunction::fpiem
     * @see fluxionum::FixedParametricIterativeEulerMethod
     */
    inline fluxionum::FixedParametricIterativeEulerMethod<double, double> &
    getFpiem() {return fpiem;}
    /**
     * @brief Obtain the last time evaluated by this function
     * @return The last time evaluated by this function
     * @see DesignTrajectoryFunction::lastTime
     */
    inline double getLastTime() const {return lastTime;}
    /**
     * @brief Set the last time evaluated by this function
     * @param lastTime The last time evaluated by this function
     * @see DesignTrajectoryFunction::lastTime
     */
    inline void setLastTime(double const lastTime) {this->lastTime = lastTime;}

};
