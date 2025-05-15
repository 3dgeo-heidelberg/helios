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
class SwapOnRepeatHandler
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief How many swaps must be handled.
   */
  int numTargetSwaps = 0;
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
  std::deque<std::deque<AbstractGeometryFilter*>> swapFilters;
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
  /**
   * @brief True if the handler requires to keep the current scene's CRS
   *  bounding box (default), False otherwise.
   */
  bool keepCRS;
  /**
   * @brief Whether the handled scene part is null (e.g., it has been
   *  discarded). It can be useful to handle null scene parts that will
   *  be recycled in the future (they are flagged as null instead of
   *  fully deleted, so they can become available later on).
   */
  bool null;

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
  void swap(ScenePart& sp);
  /**
   * @brief This method must be called after constructing a handler but
   *  before using it.
   *
   * Calling prepare will update the internal state of the handler so it is
   *  ready to handle de swap operations for the many simulation replays.
   *
   * @param sp The baseline scene part for the handler.
   */
  void prepare(ScenePart* sp);

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Get the number of swaps that must be handled in total.
   * @return Number of swaps that must be handled.
   * @see SwapOnRepeatHandler::numTargetSwaps
   */
  inline int getNumTargetSwaps() const { return numTargetSwaps; }
  /**
   * @brief Get the number of replays that are expected from this handler.
   *  Note that the final number of replays for a simulation will be the
   *  number of the replays of the handler that leads to more replays
   *  (i.e., the max).
   * @return Target number of replays.
   */
  inline int getNumTargetReplays() const { return numTargetReplays; }
  /**
   * @brief Check whether the handler has pending swaps.
   * @return True if the handler has pending swaps, false otherwise.
   */
  inline bool hasPendingSwaps() const
  {
    return numCurrentSwaps < numTargetSwaps;
  }
  /**
   * @brief Check whether the handler needs an holistic approach (all the
   *  vertices must be considered as a whole).
   *
   * Swap on repeat handlers are typically holistic if the geometry they
   *  handle was loaded from a point cloud (see XYZPointCloudFileLoader).
   * @return True if the handler needs an holistic approach, false otherwise.
   */
  inline bool isHolistic() const { return holistic; }
  /**
   * @brief Check whether the active swap of the handler is at its first
   *  play or not (when TTL>1, a swap will be active for more than one play).
   * @return True if the active swap is at its first play, false otherwise.
   */
  inline bool isOnSwapFirstPlay() const { return onSwapFirstPlay; }
  /**
   * @brief Set the flag that specifies whether the active swap is at its
   *  first simulation play.
   * @param onSwapFirstPlay The new value for the flag.
   */
  inline void setOnSwapFirstPlay(bool const onSwapFirstPlay)
  {
    this->onSwapFirstPlay = onSwapFirstPlay;
  }
  /**
   * @brief Check whether the scene part must be discarded before the next
   *  replay.
   * @return True if the scene part must be discarded before the next replay,
   *  false otherwise.
   */
  inline bool needsDiscardOnReplay() const { return discardOnReplay; }
  /**
   * @brief Set the discard on replay flag.
   * @param discardOnReplay New value for the discard on replay flag.
   * @see SwapOnRepeatHandler::discardOnReplay
   */
  inline void setDiscardOnReplay(bool const discardOnReplay)
  {
    this->discardOnReplay = discardOnReplay;
  }
  /**
   * @brief Check whether there are pending filters (True) or not (False).
   *
   * A SwapOnRepeatHandler is said to have no future if there are no further
   * filters to be applied at any point in the future. When the handler
   * needs to be discarded on replay and it has no future, it can be fully
   * deleted. Otherwise, it will be recycled in the future despite it is
   * discarded in the present.
   *
   * @see SwapOnRepeatHandler::swapFilters
   */
  inline bool hasNoFuture() const { return swapFilters.empty(); }
  /**
   * @brief Push the swap filters to the handler.
   *
   * NOTE that swap filters are executed in the same order they are given.
   *  In other words, the first sequence of swap filters is applied on the
   *  first swap operation.
   *
   * @param swapFilters The filters for the next swap operation.
   */
  void pushSwapFilters(std::deque<AbstractGeometryFilter*> const& swapFilters);
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
  std::vector<Primitive*>& getBaselinePrimitives();
  /**
   * @brief Set the keepCRS flag.
   * @param keepCRS The new keepCRS flag for the handler.
   * @see SwapOnRepeatHandler::keepCRS
   */
  inline void setKeepCRS(bool const keepCRS) { this->keepCRS = keepCRS; }
  /**
   * @brief Check whether the current keepCRS flag is enabled.
   * @return True if the handler requires to keep the current scene's CRS,
   *  False otherwise.
   * @see SwapOnRepeatHandler::keepCRS
   */
  inline bool isKeepCRS() const { return keepCRS; }
  /**
   * @brief Check whether the current null flag is enabled.
   * @return True if the handled scene part is null, False otherwise.
   * @see SwapOnRepeatHandler::null
   */
  inline bool isNull() const { return null; }
  /**
   * @brief Set the null flag.
   * @param null The new null flag for the handler.
   * @see SwapOnRepeatHandler::null
   */
  inline void setNull(bool const null) { this->null = null; }

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
  void doSwap(ScenePart& sp);
  /**
   * @brief Update the geometry of the destiny scene part with the geometry
   *  of the source scene part.
   * @param src The source scene part.
   * @param dst The destiny scene part.
   * @see SwapOnRepeatHandler::doSwap
   */
  void doGeometricSwap(ScenePart& src, ScenePart& dst);
};
