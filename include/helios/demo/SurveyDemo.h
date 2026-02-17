#ifdef PCL_BINDING
#pragma once

#include <helios/demo/BaseDemo.h>

#include <string>

namespace HeliosDemos {

using std::string;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief SurveyDemo class
 *
 * Can be overridden to implement new demos which use a survey or some of its
 *  components (for instance, a scene)
 *
 * NOTICE this is an abstract class which does not provides an implementation
 *  for the run method. In consequence, any class which extends SurveyDemo
 *  stills needing to provide an implementation of run method defining the
 *  demo behavior
 */
class SurveyDemo : BaseDemo
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Path to the survey XML file
   */
  string surveyPath;
  /**
   * @brief Path to the assets directory
   */
  string assetsPath;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Survey demo constructor
   * @param name Name for the demo
   * @param surveyPath Path to the survey XML file
   * @see BaseDemo::BaseDemo(string const)
   * @see BaseDemo::name
   * @see SurveyDemo::surveyPath
   */
  SurveyDemo(string const name,
             string const surveyPath,
             string const assetsPath)
    : BaseDemo(name)
    , surveyPath(surveyPath)
    , assetsPath(assetsPath) {};
  virtual ~SurveyDemo() = default;

  // ***  SURVEY METHODS  *** //
  // ************************ //
  /**
   * @brief Check whether the survey path points to an accessible file (true)
   *  or not (false)
   * @return True if the survey path points to an accessible file, false
   *  otherwise
   */
  bool validateSurveyPath();
  /**
   * @brief Check whether the assets path points to an accessible directory
   *  (true) or not (false)
   * @return True if assets path points to an accessible directory, false
   *  otherwise
   */
  bool validateAssetsPath();

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Get survey path
   * @return Survey path
   * @see SurveyDemo::surveyPath
   */
  string const& getSurveyPath() const { return surveyPath; }
  /**
   * @brief Set survey path
   * @param surveyPath New survey path
   * @see SurveyDemo::surveyPath
   */
  void setSurveyPath(string const surveyPath) { this->surveyPath = surveyPath; }
  /**
   * @brief Get assets path
   * @return Assets path
   * @see SurveyDemo::assetsPath
   */
  string const& getAssetsPath() const { return assetsPath; }
  /**
   * @brief Set assets path
   * @param assetsPath New assets path
   * @see SurveyDemo::assetsPath
   */
  void setAssetsPath(string const assetsPath) { this->assetsPath = assetsPath; }
};

}

#endif
