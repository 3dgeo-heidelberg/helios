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
     * @brief The time to live for each swap (in number of simulation plays).
     */
    std::deque<int> timesToLive;
    /**
     * @brief The time to live of the current scene part before doing further
     *  swaps (in number of simulation plays).
     */
    int currentTimeToLive;
    /**
     * @brief The baseline scene part before applying any transformation.
     */
    std::unique_ptr<ScenePart> baseline;
    /**
     * @brief Specify whether the scene part associated to the handler must
     *  be discarded before the next simulation play.
     */
    bool discardOnReplay;

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
     * @see SwapOnRepeatHandler::doSwap
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
    inline int getNumTargetSwaps() const {return numTargetSwaps;}
    /**
     * @brief Check whether the handler has pending swaps.
     * @return True if the handler has pending swaps, false otherwise.
     */
    inline bool hasPendingSwaps() const
    {return numCurrentSwaps < numTargetSwaps;}
    inline bool needsDiscardOnReplay() const {return discardOnReplay;}
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
    /**
     * @brief Push the time to live to its queue.
     * @param timeToLive The time to live to be pushed.
     * @see SwapOnRepeatHandler::timesToLive
     */
    void pushTimeToLive(int const timeToLive);

protected:
    // ***  UTIL METHODS  *** //
    // ********************** //
    /**
     * @brief Compute the logic of the swap operation.
     *
     * Note that this method assists the SwapOnRepeatHandler::swap method.
     * The main swap method handles the logic on whether the swap must be
     * effective or not (e.g., depending on the time to live), and the doSwap
     * method is called only when the swap must be effective.
     *
     * @param sp The scene part that needs the swap.
     * @see SwapOnRepeatHandler::swap
     */
    void doSwap(ScenePart &sp);
    /**
     * @brief Update the geometry of the destiny scene part with the geometry
     *  of the source scene part.
     * @param src The source scene part.
     * @param dst The destiny scene part.
     * @see SwapOnRepeatHandler::doSwap
     */
    void doGeometricSwap(ScenePart &src, ScenePart &dst);

};