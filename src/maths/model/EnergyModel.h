#pragma once


/**
 * @author Alberto M. Esmoris Pena
 *
 * @brief Abstract class providing the interface for any energy model.
 */
template <
    typename ReceivedPowerArgs,
    typename EmittedPowerArgs,
    typename TargetAreaArgs,
    typename CrossSectionArgs
>
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
    virtual double computeReceivedPower(ReceivedPowerArgs const & args) = 0;
    /**
     * @brief Compute the emitted power \f$P_e\f$.
     *
     * Note this method must be overriden by any concrete class providing
     * a computable energy model.
     *
     * @return The emitted power \f$P_e\f$.
     */
    virtual double computeEmittedPower(EmittedPowerArgs const & args) = 0;
    /**
     * @brief Compute the target area \f$A\f$.
     *
     * Note this method must be overriden by any concrete class providing
     * a computable energy model.
     *
     * @return The target area \f$A\f$.
     */
    virtual double computeTargetArea(TargetAreaArgs const & args) = 0;
    /**
     * @brief Compute the cross section \f$\sigma\f$.
     *
     * Note this method must be overriden by any concrete class providing
     * a computable energy model.
     *
     * @return The target area \f$\sigma\f$.
     */
    virtual double computeCrossSection(CrossSectionArgs const & args) = 0;
};