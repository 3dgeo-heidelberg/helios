#pragma once

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief The facade for FMS factories
 */
class FMSFactoryFacade
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for FMS factory facade
   */
  FMSFactoryFacade() = default;
  virtual ~FMSFactoryFacade() = default;

  // ***  FACTORY FACADE METHODS  *** //
  // ******************************** //
};

}
}
