#pragma once

#include <adt/exprtree/UnivarExprTreeNode.h>
#include <scanner/beamDeflector/PolygonMirrorBeamDeflector.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Extend the PolygonMirrorBeamDeflector to support error modeling
 *  through register-based univariate expression trees
 * @see PolygonMirrorBeamDeflector
 * @see UnivarExprTreeNode
 */
class EvalPolygonMirrorBeamDeflector : public PolygonMirrorBeamDeflector
{

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The expression tree to compute vertical angle errors as a
   *  function of vertical angle
   *
   * \f[
   *  \Delta\theta(\theta)
   * \f]
   *
   * @see UnivarExprTreeNode
   */
  std::shared_ptr<UnivarExprTreeNode<double>> vertAngErrExpr;

public:
  /**
   * @brief The exact value of the current beam angle (radians), i.e., the
   *  current beam angle withour error.
   * @see AbstractBeamDeflector::state_currentBeamAngle_rad
   */
  double state_currentExactBeamAngle_rad;
  /**
   * @brief Relative emitter attitude with no mechanical error
   */
  Rotation cached_exactEmitterRelativeAttitude =
    Rotation(glm::dvec3(1, 0, 0), 0);

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for evaluable polygon mirror beam deflector
   * @see EvalPolygonMirrorBeamDeflector::state_currentExactBeamAngle_rad
   * @see PolygonMirrorBeamDeflector
   * @see PolygonMirrorBeamDeflector::PolygonMirrorBeamDeflector
   * @see AbstractBeamDeflector
   * @see AbstractBeamDeflector::AbstractBeamDeflector
   */
  EvalPolygonMirrorBeamDeflector(
    double _scanFreqMax_Hz,
    double _scanFreqMin_Hz,
    double _scanAngleMax_rad,
    double _scanAngleEffectiveMax_rad,
    std::shared_ptr<UnivarExprTreeNode<double>> vertAngErrExpr = nullptr)
    : PolygonMirrorBeamDeflector(_scanFreqMax_Hz,
                                 _scanFreqMin_Hz,
                                 _scanAngleMax_rad,
                                 _scanAngleEffectiveMax_rad)
    , vertAngErrExpr(vertAngErrExpr)
  {
    this->state_currentExactBeamAngle_rad = this->state_currentBeamAngle_rad;
  }
  ~EvalPolygonMirrorBeamDeflector() override = default;
  std::shared_ptr<AbstractBeamDeflector> clone() override;
  void _clone(std::shared_ptr<AbstractBeamDeflector> abd) override;

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @see PolygonMirrorBeamDeflector::applySettings
   */
  void applySettings(std::shared_ptr<ScannerSettings>) override;
  /**
   * @see PolygonMirrorBeamDeflector::doSimStep
   */
  void doSimStep() override;

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @see AbstractBeamDeflector::getExactEmitterRelativeAttitude
   */
  Rotation getExactEmitterRelativeAttitude() override
  {
    return cached_exactEmitterRelativeAttitude;
  }
  /**
   * @see AbstractBeamDeflector::getCurrentExactBeamAngle
   */
  double getCurrentExactBeamAngle() override
  {
    return state_currentExactBeamAngle_rad;
  }
  /**
   * @see AbstractBeamDeflector4::hasMechanicalError
   */
  bool hasMechanicalError() override { return true; }
};
