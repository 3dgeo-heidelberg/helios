#pragma once

class ScenePart;
class AbstractGeometryFilter;

#include <deque>
#include <memory>

/**
 * @author Alberto M. Esmoris Pena
 * @brief Handler for scene parts that need a swap between simulation replays.
 * @see SimulationPlayer
 * @see ScenePart
 * @see ScenePart::sohr
 */
class SwapOnRepeatHandler {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief How many swaps must be handled.
     */
    int numTargetSwaps =  0;
    /**
     * @brief How many swaps have been handled.
     */
    int numCurrentSwaps = 0;
    /**
     * @brief The filters to be applied at each swap operation.
     */
    std::deque<std::deque<AbstractGeometryFilter *>> swapFilters;
    /**
     * @brief The baseline scene part before applying any transformation.
     */
    std::unique_ptr<ScenePart> baseline;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief SwapOnRepeatHandler constructor.
     * @see SimulationPlayer
     */
    SwapOnRepeatHandler();
    virtual ~SwapOnRepeatHandler() = default;

    // ***   MAIN METHODS   *** //
    // ************************ //
    /**
     * @brief Compute the swap on the given scene part.
     * @param sp The scene part that needs the swap.
     * @see ScenePart
     */
    void swap(ScenePart &sp);
    /**
     * @brief This method must be called after constructing a handler but
     *  before using it.
     *
     * Calling prepare will update the internal state of the handler so it is
     *  ready to handle de swap operations for the many simulation replays.
     *
     * @param sp The baseline scene part for the handler.
     */
    void prepare(std::shared_ptr<ScenePart> sp);

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Get the number of swaps that must be handled in total.
     * @return Number of swaps that must be handled.
     * @see SwapOnRepeatHandler::numTargetSwaps
     */
    inline int getNumTargetSwaps() {return numTargetSwaps;}
    /**
     * @brief Check whether the handler has pending swaps.
     * @return True if the handler has pending swaps, false otherwise.
     */
    inline bool hasPendingSwaps() {return numCurrentSwaps < numTargetSwaps;}
    /**
     * @brief Push the swap filters to the handler.
     *
     * NOTE that swap filters are executed in the same order they are given.
     *  In other words, the first sequence of swap filters is applied on the
     *  first swap operation.
     *
     * @param swapFilters The filters for the next swap operation.
     */
    void pushSwapFilters(
        std::deque<AbstractGeometryFilter *> const &swapFilters
    );

protected:
    // ***  UTIL METHODS  *** //
    // ********************** //
    /**
     * @brief Update the geometry of the destiny scene part with the geometry
     *  of the source scene part.
     * @param src The source scene part.
     * @param dst The destiny scene part.
     */
    void doGeometricSwap(ScenePart &src, ScenePart &dst);


};