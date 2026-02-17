#pragma once

#include <helios/filems/facade/FMSFacade.h>
#include <helios/sim/comps/Survey.h>

#include <memory>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Factory to build FMS facades
 */
class FMSFacadeFactory
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for FMS facade factory
   */
  FMSFacadeFactory() = default;
  virtual ~FMSFacadeFactory() = default;

  // ***  BUILD METHODS  *** //
  // *********************** //
  /**
   * @brief Build a FMS facade connected with given survey
   * @param outdir Root directory for output files
   * @param lasScale Specify LAS format scale factor
   * @param lasOutput Flag to specify LAS output format. True implies using
   *  LAS output format, false implies don't
   * @param las10 Flag to specify that the output format must be LAS v1.0.
   * @param zipOutput Flag to specify output zipping. True implies output
   *  will be zipped, false means it will not
   * @param splitByChannel Flag to specify whether the measurements must be
   *  split by channel (True) or not (False). If True, then each scanning
   *  device will be written to a different file even for the same leg.
   * @param survey The survey to which the facade must be connected
   * @return Built FMS facade
   */
  std::shared_ptr<FMSFacade> buildFacade(std::string const& outdir,
                                         double const lasScale,
                                         bool const lasOutput,
                                         bool const las10,
                                         bool const zipOutput,
                                         bool const splitByChannel,
                                         Survey& survey,
                                         bool const updateSurvey = true);
  /**
   * @brief Overload of buildFacade method that considers splitByChannel as
   *  false by default (mostly because of retrocompatibility)
   * @see FMSFacadeFactory::buildFacade
   */
  inline std::shared_ptr<FMSFacade> buildFacade(std::string const& outdir,
                                                double const lasScale,
                                                bool const lasOutput,
                                                bool const las10,
                                                bool const zipOutput,
                                                Survey& survey,
                                                bool const updateSurvey = true)
  {
    return buildFacade(outdir,
                       lasScale,
                       lasOutput,
                       las10,
                       zipOutput,
                       false,
                       survey,
                       updateSurvey);
  }
};

}
}
