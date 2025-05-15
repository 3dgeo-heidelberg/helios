#ifdef PCL_BINDING

#pragma once

#include <visualhelios/VHDynCanvas.h>
#include <visualhelios/adapters/VHStaticObjectAdapter.h>

namespace visualhelios {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract class providing some behaviors and defining the interface
 *  for all canvas which deal with normals
 */
class VHNormalsCanvas : public VHDynCanvas
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Specify if the keyboard callback to toggle normals visualization
   *  is enabled (true) or not (false)
   */
  bool normalsKeyboardCallbackEnabled;
  /**
   * @brief Specify if the normals keyboard callback usage text is enabled
   *  (true) or not (false)
   */
  bool normalsUsageTextEnabled;
  /**
   * @brief Specify if the normals canvas must render normals (true) or not
   *  (false)
   */
  bool renderingNormals;
  /**
   * @brief Specify the magnitude of normal vector for visualization
   */
  float normalMagnitude;
  /**
   * @brief Default color for normals in RGB format with components in
   *  \f$[0, 1]\f$
   */
  float normalDefColor[3];

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for the visual Helios normals canvas
   * @see visualhelios::VHDynCanvas::VHDynCanvas
   */
  VHNormalsCanvas()
    : VHNormalsCanvas("Visual Helios normals canvas")
  {
  }
  /**
   * @brief Main constructor for the visual helios normals canvas which
   *  allows for title specification, flags specification and normal
   *  magnitude specification too
   * @see visualhelios::VHNormalsCanvas::normalsKeyboardCallbackEnabled
   * @see visualhelios::VHNormalsCanvas::normalsUsageTextEnabled
   * @see visualhelios::VHNormalsCanvas::renderingNormals
   * @see visualhelios::VHNormalsCanvas::normalMagnitude
   * @see visualhelios::VHDynCanvas::VHDynCanvas(string const)
   */
  VHNormalsCanvas(string const title,
                  bool const normalsKeyboardCallbackEnabled = true,
                  bool const normalsUsageTextEnabled = true,
                  bool const renderingNormals = true,
                  float const normalMagnitude = 1.0);
  virtual ~VHNormalsCanvas() = default;

protected:
  // ***  CANVAS METHODS  *** //
  // ************************ //
  /**
   * @see VHDynCanvas::configure
   */
  void configure() override;
  /**
   * @see VHDynCanvas::start
   */
  void start() override;

  // ***  NORMALS RENDERING METHODS  *** //
  // *********************************** //
  /**
   * @brief Render normals for each primitive of given dynamic object.
   * This method implementation must be provided by concrete derived classes
   *  so the logic of the VHNormalCanvas can be computed
   * @param dynObj Dynamic object which normals will be rendered if enabled
   */
  virtual void renderNormals(VHStaticObjectAdapter& dynObj) = 0;
  /**
   * @brief Remove all rendered normals.
   * This method implementation must be provided by concrete derived classes
   *  so the logic of the VHNormalCanvas can be computed
   */
  virtual void unrenderAllNormals() = 0;
  /**
   * @brief Register the keyboard callback to toggle normals rendering
   *  on/off.
   *
   * This method is called during configure stage
   */
  virtual void registerNormalsKeyboardCallback();
  /**
   * @brief Render text explaining how to use the normals keyboard callback.
   *
   * This method is called during the start stage
   */
  virtual void addKeyboardCallbackUsageText();

public:
  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Check whether the normals canvas is rendering normals or not
   * @return True if normals canvas is rendering normals, false otherwise
   * @see visualhelios::VHNormalsCanvas::renderingNormals
   */
  virtual inline bool isRenderingNormals() const { return renderingNormals; }
  /**
   * @brief Enable or disable normals rendering
   * @param renderingNormals True to enable rendering normals, false
   *  to disable it
   * @see visualhelios::VHNormalsCanvas::renderingNormals
   */
  virtual inline void setRenderingNormals(bool const renderingNormals)
  {
    this->renderingNormals = renderingNormals;
  }
  /**
   * @brief Obtain the magnitude for normal vectors visualization
   * @return Magnitude for normal vectors visualization
   * @see visualhelios::VHNormalCanvas::normalMagnitude
   */
  virtual inline float getNormalMagnitude() const { return normalMagnitude; }
  /**
   * @brief Set the magnitude for normal vectors visualization
   * @param normalMagnitude New magnitude for normal vectors visualization
   * @see visualhelios::VHNormalCanvas::normalMagnitude
   */
  virtual inline void setNormalMagnitude(float const normalMagnitude)
  {
    this->normalMagnitude = normalMagnitude;
  }
  /**
   * @brief Check whether normals keyboard callback is enabled (true) or not
   *  (false)
   * @return True if normals keyboard callback is enabled, false otherwise
   */
  virtual inline bool isNormalsKeyboardCallbackEnabled() const
  {
    return normalsKeyboardCallbackEnabled;
  }
  /**
   * @brief Check whether normals usage text is enabled (true) or not (false)
   * @return True if normals usage text is enabled, false otherwise
   */
  virtual inline bool isNormalsUsageTextEnabled() const
  {
    return normalsUsageTextEnabled;
  }
  /**
   * @brief Return the default color for normals visualization
   * @return Default color for normals visualization
   * @see visualhelios::VHNormalCanvas::normalDefColor
   */
  virtual inline float const* getNormalDefColor() const
  {
    return normalDefColor;
  }
  /**
   * @brief Set the default color for normals visualization
   * @param color Default color for normals visualization
   * @see visualhelios::VHNormalCanvas::normalDefColor
   */
  virtual inline void setNormalDefColor(float const color[3])
  {
    for (size_t i = 0; i < 3; ++i)
      normalDefColor[i] = color[i];
  }
  /**
   * @brief Set the default color for normals visualization
   * @param r Red component for normals default color
   * @param g Green component for normals default color
   * @param b Blue component for normals default color
   */
  virtual inline void setNormalDefColor(float const r,
                                        float const g,
                                        float const b)
  {
    normalDefColor[0] = r;
    normalDefColor[1] = g;
    normalDefColor[2] = b;
  }
};

}

#endif
