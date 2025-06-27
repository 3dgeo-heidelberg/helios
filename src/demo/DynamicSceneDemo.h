#ifdef PCL_BINDING
#pragma once

#include <Survey.h>
#include <demo/SurveyDemo.h>
#include <visualhelios/VHSceneCanvas.h>
#include <visualhelios/adapters/VHDynObjectXYZAdapter.h>

#include <string>

namespace HeliosDemos {

using visualhelios::VHDynObjectXYZAdapter;
using visualhelios::VHSceneCanvas;

using std::shared_ptr;
using std::string;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Dynamic scene demo
 *
 * This demo implements the rendering of a given dynamic scene
 *
 * @see HeliosDemos::SurveyDemo
 */
class DynamicSceneDemo : public SurveyDemo
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Used to store the dynamic scene wrapper that must be used to
   *  render non dynamic scenes
   */
  shared_ptr<DynScene> dsWrapper = nullptr;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Dynamic scene demo constructor
   * @see HeliosDemos::SurveyDemo::SurveyDemo
   */
  DynamicSceneDemo(string const surveyPath, string const assetsPath)
    : DynamicSceneDemo("Dynamic scene demo", surveyPath, assetsPath)
  {
  }
  /**
   * @brief Dynamic scene demo constructor
   * @see HeliosDemos::SurveyDemo::SurveyDemo
   */
  DynamicSceneDemo(string const name,
                   string const surveyPath,
                   string const assetsPath)
    : SurveyDemo(name, surveyPath, assetsPath)
  {
  }
  virtual ~DynamicSceneDemo() = default;

  // ***   R U N   *** //
  // ***************** //
  /**
   * @see BaseDemo::run
   */
  void run() override;

  // ***   U T I L S   *** //
  // ********************* //
  /**
   * @brief Validate survey and assets path. If any of them is not valid a
   *  proper exception is thrown
   */
  void validatePaths();
  /**
   * @brief Load survey containing the scene to be rendered
   * @return Loaded survey containing the scene to be rendered
   */
  virtual shared_ptr<Survey> loadSurvey();
  /**
   * @brief Build a visual Helios scene canvas from given survey.
   *
   * If the survey contains a dynamic scene it will be used. If it contains
   *  a basic scene, it will be wrapped by a dynamic scene so it can be
   *  rendered too. Notice however that, in the last case, no dynamic
   *  behavior will take place at all as the scene itself is not really
   *  dynamic.
   *
   * @param survey Survey which must contain a valid scene to build
   *  the visual Helios scene canvas
   * @return Built visual Helios scene canvas
   */
  shared_ptr<VHSceneCanvas> buildCanvas(shared_ptr<Survey> survey);
};

}

#endif
