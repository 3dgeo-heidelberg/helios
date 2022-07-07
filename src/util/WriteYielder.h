#pragma once

#include <filems/facade/FMSWriteFacade.h>
#include <util/Yielder.h>

using helios::filems::FMSWriteFacade;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract class representing a write yielder. It is, a yielder which
 *  can use the FMSWriteFacade to write its output.
 * @tparam T The type of object handled by the yielder
 * @see helios::filems::FMSWriteFacade
 */
template <typename T>
class WriteYielder : public Yielder<T>{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The facade for writing operations
     * @see filems::FMSWriteFacade
     */
    FMSWriteFacade &write;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for the abstract write yielder
     * @see Yielder::Yielder
     * @see WriteYielder::write
     */
    WriteYielder(
        FMSWriteFacade &write,
        size_t bufferSize=256
    ) :
        Yielder<T>(bufferSize),
        write(write)
    {}
    virtual ~WriteYielder() = default;
};