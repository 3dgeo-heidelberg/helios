#pragma once
#include <maths/model/ModelArg.h>
#include <vector>


/**
 * @author Alberto M. Esmoris Pena
 *
 * @brief Abstract class providing the interface for any energy model.
 */
class EnergyModel{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    EnergyModel() = default;
    virtual ~EnergyModel() = default;

    // ***  METHODS  *** //
    // ***************** //
    /**
     * @brief Compute the received power \f$P_r\f$.
     *
     * Note this method must be overriden by any concrete class providing
     * a computable energy model.
     *
     * @return The received power \f$P_r\f$.
     */
    virtual double computeReceivedPower(
        ModelArg const & args
#if DATA_ANALYTICS >=2
       ,std::vector<std::vector<double>> &calcIntensityRecords
#endif
    ) = 0;
    /**
     * @brief Compute the emitted power \f$P_e\f$.
     *
     * Note this method must be overriden by any concrete class providing
     * a computable energy model.
     *
     * @return The emitted power \f$P_e\f$.
     */
    virtual double computeEmittedPower(ModelArg const & args) = 0;
    /**
     * @brief Compute the target area \f$A\f$.
     *
     * Note this method must be overriden by any concrete class providing
     * a computable energy model.
     *
     * @return The target area \f$A\f$.
     */
    virtual double computeTargetArea(
        ModelArg const & args
#if DATA_ANALYTICS >=2
       ,std::vector<std::vector<double>> &calcIntensityRecords
#endif
    ) = 0;
    /**
     * @brief Compute the cross section \f$\sigma\f$.
     *
     * Note this method must be overriden by any concrete class providing
     * a computable energy model.
     *
     * @return The target area \f$\sigma\f$.
     */
    virtual double computeCrossSection(ModelArg const & args) = 0;
};