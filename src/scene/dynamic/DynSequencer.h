#pragma once

#include <scene/dynamic/DynSequence.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @tparam T Type of elements composing sequences
 *
 * @brief Dynamic sequencer handles dynamic sequences in a sequential fashion
 *
 * @see DynSequence
 */
template<typename T>
class DynSequencer
{
private:
  // *********************** //

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The dynamic sequences handled by the dynamic sequencer
   */
  std::unordered_map<std::string, std::shared_ptr<DynSequence<T>>> dynseqs;
  /**
   * @brief The start dynamic sequence
   */
  std::shared_ptr<DynSequence<T>> start;
  /**
   * @brief The current dynamic sequence
   */
  std::shared_ptr<DynSequence<T>> current;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for the dynamic sequencer
   */
  DynSequencer()
    : start(nullptr)
    , current(nullptr)
  {
  }
  virtual ~DynSequencer() {}

  // ***  DYNAMIC SEQUENCING   *** //
  // ***************************** //
  /**
   * @brief Handle next sequential behavior
   * @return Sequence for the step
   */
  virtual std::vector<std::shared_ptr<T>> nextStep();
  /**
   * @brief Check if the dynamic sequencer supports a next step or not
   * @return True if dynamic sequencer supports a next step, false otherwise
   */
  virtual inline bool hasNextStep() { return current != nullptr; }

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Release all the resources that belong to the dynamic sequencer.
   *  Note that calling release implies that the scene part can no longer be
   *  used.
   */
  virtual void release();

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Add the dynamic sequence to the sequencer.
   *
   * NOTICE the first added dynamic sequence will become the start dynamic
   *  sequence by default
   *
   * @param dynseq Dynamic sequence to be added
   */
  void add(std::shared_ptr<DynSequence<T>> dynseq);
  /**
   * @brief Remove the dynamic sequence with given identifier
   * @param id Identifier of dynamic sequence to be removed
   */
  inline void remove(std::string const& id) { dynseqs.erase(dynseqs.find(id)); }
  /**
   * @brief Get the dynamic sequence with given identifier
   * @param id Identifier of dynamic sequence to be retrieved
   * @return Requested dynamic sequence if any, nullptr otherwise
   */
  std::shared_ptr<DynSequence<T>> get(std::string const& id);
  /**
   * @brief Check if the dynamic sequencer has a dynamic sequence with given
   *  identifier
   * @param id Identifier of the dynamic sequence to be checked
   * @return True if there is a dynamic sequence with given identifier, false
   *  otherwise
   */
  inline bool has(std::string const& id)
  {
    return dynseqs.find(id) != dynseqs.end();
  }
  /**
   * @brief Remove all dynamic sequences composing the dynamic sequencer
   */
  inline void clear() { dynseqs.clear(); }
  /**
   * @brief Obtain a read-write reference to the dynamic sequences.
   *
   * Using this method is strictly discouraged. It must be used (with
   * caution) only when it is strictly necessary
   *
   * @return All the dynamic sequences
   */
  inline std::unordered_map<std::string, std::shared_ptr<DynSequence<T>>>&
  getAllSequencesByRef()
  {
    return dynseqs;
  }
};

#include <scene/dynamic/DynSequencer.tpp>
