#pragma once

class Primitive;
class ScenePart;
class AbstractGeometryFilter;

#include <deque>
#include <memory>
#include <vector>

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
     * @brief How many replays are expected by this handler.
     */
    int numTargetReplays = 0;
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
     * @brief Specify whether the scene part associated to the handler must
     *  be discarded before the next simulation play.
     */
    bool discardOnReplay;
    /**
     * @brief Whether all the vertices defining each primitive must be
     *  considered as a whole.
     */
    bool holistic;
    /**
     * @brief Whether the simulation is at the first play of the current
     *  swap.
     */
    bool onSwapFirstPlay;

public:
    /**
     * @brief The baseline scene part before applying any transformation.
     */
    std::unique_ptr<ScenePart> baseline;

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
    void prepare(ScenePart * sp);

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Get the number of swaps that must be handled in total.
     * @return Number of swaps that must be handled.
     * @see SwapOnRepeatHandler::numTargetSwaps
     */
    inline int getNumTargetSwaps() const {return numTargetSwaps;}
    /**
     * @brief Get the number of replays that are expected from this handler.
     *  Note that the final number of replays for a simulation will be the
     *  number of the replays of the handler that leads to more replays
     *  (i.e., the max).
     * @return Target number of replays.
     */
    inline int getNumTargetReplays() const {return numTargetReplays;}
    /**
     * @brief Check whether the handler has pending swaps.
     * @return True if the handler has pending swaps, false otherwise.
     */
    inline bool hasPendingSwaps() const
    {return numCurrentSwaps < numTargetSwaps;}
    /**
     * @brief Check whether the handler needs an holistic approach (all the
     *  vertices must be considered as a whole).
     *
     * Swap on repeat handlers are typically holistic if the geometry they
     *  handle was loaded from a point cloud (see XYZPointCloudFileLoader).
     * @return True if the handler needs an holistic approach, false otherwise.
     */
    inline bool isHolistic() const {return holistic;}
    /**
     * @brief Check whether the active swap of the handler is at its first
     *  play or not (when TTL>1, a swap will be active for more than one play).
     * @return True if the active swap is at its first play, false otherwise.
     */
    inline bool isOnSwapFirstPlay() const {return onSwapFirstPlay;}
    /**
     * @brief Set the flag that specifies whether the active swap is at its
     *  first simulation play.
     * @param onSwapFirstPlay The new value for the flag.
     */
    inline void setOnSwapFirstPlay(bool const onSwapFirstPlay)
    {this->onSwapFirstPlay = onSwapFirstPlay;}
    /**
     * @brief Check whether the scene part must be discarded before the next
     *  replay.
     * @return True if the scene part must be discarded before the next replay,
     *  false otherwise.
     */
    inline bool needsDiscardOnReplay() const {return discardOnReplay;}
    /**
     * @brief Set the discard on replay flag.
     * @param discardOnReplay New value for the discard on replay flag.
     * @see SwapOnRepeatHandler::discardOnReplay
     */
    inline void setDiscardOnReplay(bool const discardOnReplay)
    {this->discardOnReplay = discardOnReplay;}
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
    /**
     * @brief Obtain a vector of pointers to the primitives representing the
     *  baseline of the handler.
     * @return Vector of pointers to the primitives representing the handler's
     *  baseline.
     * @see SwapOnRepeatHandler::mPrimitives
     */
    std::vector<Primitive *> & getBaselinePrimitives();

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