#ifdef PCL_BINDING

#pragma once

#include <helios/visualhelios/VHCanvas.h>

namespace visualhelios {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Visual Helios Dynamic Canvas is a class which provides the basis
 *  layer to deal with dynamic objects rendering in a general purpose fashion
 */
class VHDynCanvas : public VHCanvas
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Control whether an update is needed even when dynamic objects
   *  themselves have not been updated (true) or not (false)
   */
  bool needsUpdate = false;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for the visual Helios dynamic canvas
   * @see visualhelios::VHCanvas::VHCanvas
   */
  VHDynCanvas()
    : VHDynCanvas("Visual Helios dynamic canvas")
  {
  }
  /**
   * @brief Constructor for the visual Helios dynamic canvas which allows for
   *  title specification
   * @param title Title for the visualizer
   * @see visualhelios::VHCanvas::VHCanvas(string const)
   */
  VHDynCanvas(string const title)
    : VHCanvas(title)
  {
  }
  virtual ~VHDynCanvas() = default;

protected:
  // ***  CANVAS METHODS  *** //
  // ************************ //
  /**
   * @brief Defines the default post update behavior for dynamic canvas
   *  and derived classes.
   *
   * The dynamic canvas uses postUpdate method to disable the need for
   *  updates from canvas side by default after update has been called. It
   *  is expected that if updates are needed, they are fully computed after
   *  one call of update method. This default behavior has been implemented
   *  because it feels quite intuitive to avoid forcing full updates at
   *  each step. Otherwise, efficient implementations will be impaired by
   *  default because selective update of rendered objects will be
   *  compromised.
   *
   * In case the default behavior should be changed to the opposite, it could
   *  be easily done by overriding the postUpdate method and adding following
   *  peace of code inside:
   *
   * \code
   *  void postUpdate() override{
   *      setNeedsUpdate(true);
   *  }
   * \endcode
   *
   * @see VHCanvas::postUpdate
   */
  void postUpdate() override;

public:
  // *** GETTERs and SETTERs  *** //
  // **************************** //
  /**
   * @brief Check whether the dynamic canvas needs an update no matter what
   *  (true) or not (false)
   * @return True if the dynamic canvas needs an update, false otherwise
   */
  virtual inline bool isNeedingUpdate() const { return needsUpdate; }
  /**
   * @brief Specify if the dynamic canvas needs an update no matter what
   *  (true) or not (false)
   * @param needsUpdate True to specify the dynamic canvas needs an update,
   *  false to specify it does not
   */
  virtual inline void setNeedsUpdate(bool const needsUpdate)
  {
    this->needsUpdate = needsUpdate;
  }
};
}

#endif
