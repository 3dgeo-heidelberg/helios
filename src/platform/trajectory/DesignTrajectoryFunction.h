#pragma once

#include <platform/trajectory/TrajectoryFunction.h>

#include <armadillo>


/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Abstract class representing a trajectory function that comes from
 *  interpolating a TemporalDesignMatrix and its corresponding DiffDesignMatrix
 *
 * @see TrajectoryFunction
 * @see fluxionum::TemporalDesignMatrix
 * @see fluxionum::DiffDesignMatrix
 */
class DesignTrajectoryFunction : public TrajectoryFunction {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The time frontiers
     * @see InterpolatedMovingPlatform::timeFrontiers
     */
    arma::Col<double> const &timeFrontiers;
    /**
     * @brief The values at the frontiers
     * @see InterpolatedMovingPlatform::frontierValues
     */
    arma::Mat<double> const &frontierValues;
    /**
     * @brief The derivatives at the frontiers
     * @see InterpolatedMovingPlatform::frontierDerivatives
     */
    arma::Mat<double> const &frontierDerivatives;

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
        timeFrontiers(timeFrontiers),
        frontierValues(frontierValues),
        frontierDerivatives(frontierDerivatives)
    {}
    virtual ~DesignTrajectoryFunction() = default;

};
