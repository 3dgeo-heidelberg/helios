#pragma once

#include <platform/trajectory/DesignTrajectoryFunction.h>
#include <fluxionum/ParametricClosestLesserSampleFunction.h>
#include <fluxionum/FixedParametricIterativeEulerMethod.h>

#include <armadillo>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class representing a design trajectory function obtained through
 *  the interpolation of a parametric function describing the position
 *
 * @see DesignTrajectoryFunction
 */
class PositionTrajectoryFunction : public DesignTrajectoryFunction{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Index of the column specifying the \f$x\f$ coordinate in the
     *  frontier matrices for both the values and their derivatives
     *
     * By default it is the \f$0\f$ column
     *
     * @see DesginTrajectoryFunction::frontierValues
     * @see DesginTrajectoryFunction::frontierDerivatives
     */
    size_t xIdx;
    /**
     * @brief Index of the column specifying the \f$y\f$ coordinate in the
     *  frontier matrices for both the values and their derivatives
     *
     * By default it is the \f$1\f$ column
     *
     * @see DesginTrajectoryFunction::frontierValues
     * @see DesginTrajectoryFunction::frontierDerivatives
     */
    size_t yIdx;
    /**
     * @brief Index of the column specifying the \f$z\f$ coordinate in the
     *  frontier matrices for both the values and their derivatives
     *
     * By default it is the \f$2\f$ column
     *
     * @see DesginTrajectoryFunction::frontierValues
     * @see DesginTrajectoryFunction::frontierDerivatives
     */
    size_t zIdx;
    /**
     * @brief The function approximating the position's derivative at a given
     *  time
     * \f[
     *  \frac{d\vec{p}}{dt} = \left(
     *      \frac{dx}{dt}, \frac{dy}{dt}, \frac{dz}{dy}
     *  \right)
     * \f]
     */
    ParametricClosestLesserSampleFunction<double, double> pclsf;
    /**
     * @brief The function solving the Euler method to estimate the parametric
     *  function defining the position
     * \f[
     *  \vec{p}(t+h) = \vec{p}(t) + h \frac{d\vec{p}}{dt}
     * \f]
     */
    FixedParametricIterativeEulerMethod<double, double> fpiem;
    /**
     * @brief The last time value evaluated by this function. By default, it is
     *  0
     */
    double lastTime;


public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for PositionTrajectoryFunction
     * @see DesignTrajectoryFunction::DesignTrajectoryFunction
     * @see PositionTrajectoryFunction::xIdx
     * @see PositionTrajectoryFunction::yIdx
     * @see PositionTrajectoryFunction::zIdx
     */
    PositionTrajectoryFunction(
        arma::Col<double> const &timeFrontiers,
        arma::Mat<double> const &frontierValues,
        arma::Mat<double> const &frontierDerivatives,
        size_t const xIdx=0,
        size_t const yIdx=1,
        size_t const zIdx=2
    ) :
        DesignTrajectoryFunction(
            timeFrontiers, frontierValues, frontierDerivatives
        ),
        xIdx(xIdx),
        yIdx(yIdx),
        zIdx(zIdx),
        pclsf(timeFrontiers, frontierDerivatives, 0),
        fpiem(
            pclsf,
            timeFrontiers(0),
            arma::Mat<double>(frontierValues.cols(
                arma::uvec({xIdx, yIdx, zIdx})
            )).row(0).as_col(),
            timeFrontiers,
            frontierValues.cols(arma::uvec({xIdx, yIdx, zIdx})),
            0
        ),
        lastTime(0)
    {}
    virtual ~PositionTrajectoryFunction() = default;

    // ***  TRAJECTORY FUNCTION  *** //
    // ***************************** //
    /**
     * @brief Evaluate the function \f$f(t) = (x_t, y_t, z_t)\f$
     * @param t The value defining a concrete time
     * @return The output as column vector \f$(x_t, y_t, z_t)^T\f$
     */
    arma::Col<double> eval(double const &t) override{
        double const h = t-lastTime;
        lastTime = t;
        return fpiem(h);
    }

};