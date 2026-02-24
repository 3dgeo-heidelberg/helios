#pragma once

#include <scene/dynamic/DynMovingObject.h>
#include <scene/dynamic/DynSequencer.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Implementation of a dynamic object which supports sequentiable
 *  dynamic motions.
 *
 * Sequentiable dynamic motions use sequences to modify motion queues so
 *  at each step queues are filled with corresponding dynamic motions from
 *  dynamic sequencers.
 *
 * @see DynMovingObject
 * @see DynSequencer
 */
class DynSequentiableMovingObject : public DynMovingObject
{
private:
  // *********************** //

protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Dynamic motion sequencer
   */
  DynSequencer<DynMotion> dmSequencer;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @see DynMovingObject::DynMovingObject
   */
  DynSequentiableMovingObject() = default;
  /**
   * @see DynMovingObject::DynMovingObject(ScenePart const &sp, bool const)
   */
  DynSequentiableMovingObject(ScenePart const& sp,
                              bool const shallowPrimitives = false)
    : DynMovingObject(sp, shallowPrimitives)
  {
  }
  /**
   * @see DynMovingObject::DynMovingObject(std::string const)
   */
  DynSequentiableMovingObject(std::string const id)
    : DynMovingObject(id)
  {
  }
  /**
   * @see DynMovingObject::DynMovingObject(std::vector<Primitive *> const &)
   */
  DynSequentiableMovingObject(std::vector<Primitive*> const& primitives)
    : DynMovingObject(primitives)
  {
  }
  /**
   * @see DynMovingObject(string const, vector<Primitive *> const &)
   */
  DynSequentiableMovingObject(std::string const id,
                              std::vector<Primitive*> const& primitives)
    : DynMovingObject(id, primitives)
  {
  }
  virtual ~DynSequentiableMovingObject() = default;

  // ***  DYNAMIC BEHAVIOR  *** //
  // ************************** //
  /**
   * @brief Sequentiable dynamic motions behavior implementation
   *
   * It is basically as the DynMovingObject::doSimStep but filling motion
   *  queues with dynamic motions coming from dynamic sequencer
   *
   * @return True if the dynamic object was modified, false otherwise
   * @see DynMovingObject::doSimStep
   * @see DynSequencer
   * @see DynSequence
   */
  bool doSimStep() override;

protected:
  /**
   * @brief Fill motion queues with dynamic motions coming from dynamic
   *  sequencer
   */
  void fillMotionQueues();

public:
  // ***   U T I L S   *** //
  // ********************* //
  /**
   * @brief Translate all the dynamic motions which are translations and
   *  have the autoCRS flag to true
   *
   * For instance, let \f$\pmb{v} = (v_x, v_y, v_z)\f$ be a translation
   *  vector. After DynSequentiableMovingObject::applyAutoCRS is called,
   *  the translation vector will be updated to be
   *  \f$\pmb{v}' = (v_x + x, v_y + y, v_z + z)\f$. Besides, the autoCRS
   *  flag will be set to false because the CRS has been applied.
   *
   *
   * @param x The CRS translation magnitude for the \f$x\f$ coordinate
   * @param y The CRS translation magnitude for the \f$y\f$ coordinate
   * @param z The CRS translation magnitude for the \f$z\f$ coordinate
   *
   * @see DynMotion::autoCRS
   */
  void applyAutoCRS(double const x, double const y, double const z);

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @see ScenePart::release
   */
  void release() override;

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  /**
   * @brief Add the dynamic sequence of dynamic motions to the dynamic motion
   *  sequencer
   * @param dmSequence Dynamic motion sequence to be added
   * @see DynSequencer::add
   */
  inline void addSequence(std::shared_ptr<DynSequence<DynMotion>> dmSequence)
  {
    return dmSequencer.add(dmSequence);
  }
  /**
   * @brief Remove the dynamic sequence of dynamic motions with given
   *  identifier
   * @param id Identifier of dynamic sequence of dynamic motions to be
   *  removed
   * @see DynSequencer::remove
   */
  inline void removeSequence(std::string const& id)
  {
    return dmSequencer.remove(id);
  }
  /**
   * @brief Get the dynamic sequence of dynamic motions with given identifier
   * @param id Identifier of dynamic sequence of dynamic motions to be
   *  retrieved
   * @return Requested dynamic sequence of dynamic motions if any, nullptr
   *  otherwise
   * @see DynSequencer::get
   */
  inline std::shared_ptr<DynSequence<DynMotion>> getSequence(
    std::string const& id)
  {
    return dmSequencer.get(id);
  }
  /**
   * @brief Check if the dynamic sequencer has a dynamic sequence of dynamic
   *  motions with given identifier
   * @param id Identifier of the dynamic sequence of dynamic motions to be
   *  checked
   * @return True if there is a dynamic sequence of dynamic motions with
   *  given identifier, false otherwise
   * @see DynSequencer::has
   */
  inline bool hasSequences(std::string const& id)
  {
    return dmSequencer.has(id);
  }
  /**
   * @brief Remove all dynamic sequences of dynamic motions composing the
   *  dynamic sequencer
   * @see DynSequencer::clear
   */
  inline void clearSequences() { dmSequencer.clear(); }
};
