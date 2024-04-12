#pragma once

#include <AbstractGeometryFilter.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief When run, the null filter will enable the discard operation on scene
 *  parts that must be swapped for null (in the context of swap on repeat).
 *
 * @see SimulationPlayer
 * @see SimulationPlayer::restartScene
 * @see SwapOnRepeatHandler
 * @see SwapOnRepeatHandler::doSwap
 */
class NullGeometryFilter : public AbstractGeometryFilter{
public:
    // ***  CONSTRUCTION  *** //
    // ********************** //
    /**
     * @brief Constructor for Null geometry filter
     * @param sp The scene part whose discard flag will be enabled.
     * @see AbstractGeometryFilter::AbstractGeometryFilter(ScenePart *)
     */
    NullGeometryFilter(ScenePart *sp) : AbstractGeometryFilter(sp) {}

    // ***  MAIN METHODS  *** //
    // ********************** //
    /**
     * @see AbstractGeometryFilter::run
     */
    ScenePart *run();

};