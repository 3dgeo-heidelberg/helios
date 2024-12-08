#pragma once

#include <scanner/ScannerHead.h>

#include <adt/exprtree/UnivarExprTreeNode.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Extend the ScannerHead to support error modeling through
 *  register-based univariate expression trees
 * @see ScannerHead
 * @see UnivarExprTreeNode
 */
class EvalScannerHead : public ScannerHead{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The expression tree to compute horizontal angle errors as a
     *  function of vertical/deflection angle
     *
     * \f[
     *  \Delta\varphi(\theta)
     * \f]
     * @see EvalScannerHead::deflectorAngle
     */
    std::shared_ptr<UnivarExprTreeNode<double>> horizAngErrExpr;
    /**
     * @brief The angular threshold to avoid \f$\sin(\theta) = 0\f$ because
     *  it will be used to divide. It is also necessary to avoid
     *  \f$\sin(\theta) \approx 0\f$ because then it will lead to divide by
     *  a very small number which could flaw the error simulation.
     * @see TrigoTricks::clipZeroSinRadians
     */
    double zeroSinThreshold_rad = 0.0;
    /**
     * @brief Pointer to the deflector's angle \f$\theta\f$ (in radians) that
     *  must be used as input for the horizontal angle error expression.
     * @see EvalScannerHead::horizAngErrExpr
     */
    double *deflectorAngle = nullptr;
    /**
     * @brief The exact current rotation angle (radians).
     * @see ScannerHead::state_currentRotateAngle_rad
     */
    double state_currentExactRotateAngle_rad = 0;
    /**
     * @brief Relative scanner head mount attitude with no mechanical error
     */
    Rotation cached_exactMountRelativeAttitude =
        Rotation(glm::dvec3(0, 1, 0), 0);

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Scanner head constructor
     * @see ScannerHead::cfg_device_rotateAxis
     * @see ScannerHead::cfg_device_rotatePerSecMax_rad
     */
    EvalScannerHead(
        glm::dvec3 headRotationAxis,
        double headRotatePerSecMax_rad,
        std::shared_ptr<UnivarExprTreeNode<double>> horizAngErrExpr=nullptr,
        double zeroSinThreshold_rad=0
    ) :
        ScannerHead(headRotationAxis, headRotatePerSecMax_rad),
        horizAngErrExpr(horizAngErrExpr),
        zeroSinThreshold_rad(zeroSinThreshold_rad),
        deflectorAngle(nullptr)
    {
        state_currentExactRotateAngle_rad = state_currentRotateAngle_rad;
    }
    ~EvalScannerHead() override {}

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @see ScannerHead::doSimStep
     */
    void doSimStep(double pulseFreq_Hz) override;

    /**
     * @see ScannerHead::rotateCompleted
     */
    bool rotateCompleted() override;

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @see ScannerHead::getExactMountRelativeAttitude
     */
    Rotation getExactMountRelativeAttitude() override;
    /**
     * @brief Set the current rotation angle.
     * @param angle_rad New current rotation angle (radians).
     * @see ScannerHead::state_currentRotateAngle_rad
     */
    void setCurrentRotateAngle_rad(double angle_rad) override;
    /**
     * @brief Set the pointer to the deflector angle associated to this head.
     * @param deflectorAngle Pointer to the new deflector angle associated to
     *  this head.
     */
    void setDeflectorAnglePtr(double *deflectorAngle)
    {this->deflectorAngle = deflectorAngle;}
    /**
     * @brief Obtain the pointer to the deflector angle associated to this head
     * @return Pointer to the deflector angle associated to this head
     */
    double * getDeflectorAnglePtr(){return deflectorAngle;}
    /**
     * @brief Obtain the deflector angle associated to this head
     * @return Deflector angle associated to this head
     */
    double getDeflectorAngle(){return *deflectorAngle;}
    /**
     * @see ScannerHead::getExactRotateCurrent
     */
    double getExactRotateCurrent() override
    {return state_currentExactRotateAngle_rad;}
    /**
     * @see ScannerHead::hasMechanicalError
     */
    bool hasMechanicalError() override {return true;}

};