#pragma once

#include <cstdlib>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Interface declaring behaviors that must be implemented by any
 *  class capable of handling completion of shared sub-tasks
 * @see SharedSubTask
 * @see SharedTaskSequencer
 */
class SharedSubTaskCompletionHandler
{
public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for the shared task completion handler
   */
  SharedSubTaskCompletionHandler() = default;
  virtual ~SharedSubTaskCompletionHandler() {}

  // ***  SHARED SUB-TASK COMPLETION HANDLING  *** //
  // ********************************************* //
  /**
   * @brief Pure virtual method that must be overridden by any concrete class
   *  to provide handling mechanism for completed shared sub-tasks.
   *
   * It is expected that this method is called always that a shared sub-task
   *  is finished.
   *
   * @param key Key identifying the completed shared sub-task inside its
   *  associated shared sub-task sequencer context
   *
   * @see SharedTaskSequencer
   * @see SharedSubTask
   */
  virtual void onSharedSubTaskCompletion(std::size_t const key) = 0;
};
