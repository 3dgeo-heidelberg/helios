#pragma once

#include <filems/facade/FMSFactoryFacade.h>
#include <filems/facade/FMSReadFacade.h>
#include <filems/facade/FMSWriteFacade.h>

#include <memory>

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief The main facade for file management system
 */
class FMSFacade
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The facade for file management system factories
   */
  FMSFactoryFacade factory;
  /**
   * @brief The facade for file management system reading operations
   */
  FMSReadFacade read;
  /**
   * @brief The facade for file management system writing operations
   */
  FMSWriteFacade write;
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief File management system facade default constructor
   */
  FMSFacade() = default;
  virtual ~FMSFacade() = default;

  // ***  FACADE LIFECYCLE METHODS  *** //
  // ********************************** //
  /**
   * @brief Disconnects all connected components from the main facade
   * @see filems::FMSFacade::disconnect
   */
  virtual void disconnect() { write.disconnect(); }
};

}
}
