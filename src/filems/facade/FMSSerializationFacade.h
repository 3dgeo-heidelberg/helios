#pragma once

namespace helios {
namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief The facade for FMS serialization
 */
class FMSSerializationFacade
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for FMS serialization facade
   */
  FMSSerializationFacade() = default;
  virtual ~FMSSerializationFacade() = default;

  // ***  SERIALIZATION FACADE METHODS  *** //
  // ************************************** //
};

}
}
