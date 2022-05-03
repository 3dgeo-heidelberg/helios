#pragma once

namespace helios{ namespace filems{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief The facade for FMS reading
 */
class FMSReadFacade{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for FMS read facade
     */
    FMSReadFacade() = default;
    virtual ~FMSReadFacade() = default;

    // ***  FACTORY READ METHODS  *** //
    // ****************************** //
};

}}
