#pragma once

#include <scene/dynamic/DynMovingObject.h>
#include <scene/dynamic/DynSequencer.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Implementation of a dynamic object which supports sequentiable
 *  rigid motions.
 *
 * Sequentiable rigid motions use sequences to modify motion queues so
 *  at each step queues are filled with corresponding rigid motions from
 *  dynamic sequencers.
 *
 * @see DynMovingObject
 * @see DynSequencer
 */
class DynSequentiableMovingObject : public DynMovingObject{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Rigid motion sequencer
     */
    DynSequencer<RigidMotion> rmSequencer;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @see DynMovingObject::DynMovingObject
     */
    DynSequentiableMovingObject() = default;
    /**
     * @see DynMovingObject::DynMovingObject(ScenePart const &sp)
     */
    DynSequentiableMovingObject(ScenePart const &sp) : DynMovingObject(sp) {}
    /**
     * @see DynMovingObject::DynMovingObject(string const)
     */
    DynSequentiableMovingObject(string const id) : DynMovingObject(id) {}
    /**
     * @see DynMovingObject::DynMovingObject(vector<Primitive *> const &)
     */
    DynSequentiableMovingObject(vector<Primitive *> const &primitives) :
        DynMovingObject(primitives)
    {}
    /**
     * @see DynMovingObject(string const, vector<Primitive *> const &)
     */
    DynSequentiableMovingObject(
        string const id,
        vector<Primitive *> const &primitives
    ) : DynMovingObject(id, primitives)
    {}
    virtual ~DynSequentiableMovingObject() = default;

    // ***  DYNAMIC BEHAVIOR  *** //
    // ************************** //
    /**
     * @brief Sequentiable rigid motions behavior implementation
     *
     * It is basically as the DynMovingObject::doStep but filling motion
     *  queues with rigid motions coming from dynamic sequencer
     *
     * @return True if the dynamic object was modified, false otherwise
     * @see DynMovingObject::doStep
     * @see DynSequencer
     * @see DynSequence
     */
    bool doStep() override;
protected:
    /**
     * @brief Fill motion queues with rigid motions coming from dynamic
     *  sequencer
     */
    void fillMotionQueues();

public:
    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Add the dynamic sequence of rigid motions to the rigid motion
     *  sequencer
     * @param rmSequence Rigid motion sequence to be added
     * @see DynSequencer::add
     */
    inline void addSequence(shared_ptr<DynSequence<RigidMotion>> rmSequence)
    {return rmSequencer.add(rmSequence);}
    /**
     * @brief Remove the dynamic sequence of rigid motions with given
     *  identifier
     * @param id Identifier of dynamic sequence of rigid motions to be removed
     * @see DynSequencer::remove
     */
    inline void removeSequence(string const & id)
    {return rmSequencer.remove(id);}
    /**
     * @brief Get the dynamic sequence of rigid motions with given identifier
     * @param id Identifier of dynamic sequence of rigid motions to be
     *  retrieved
     * @return Requested dynamic sequence of rigid motions if any, nullptr
     *  otherwise
     * @see DynSequencer::get
     */
    inline shared_ptr<DynSequence<RigidMotion>> getSequence(string const &id)
    {return rmSequencer.get(id);}
    /**
     * @brief Check if the dynamic sequenccer has a dynamic sequence of rigid
     *  motions with given identifier
     * @param id Identifier of the dynamic sequence of rigid motions to be
     *  checked
     * @return True if there is a dynamic sequence of rigid motions with
     *  given identifier, false otherwise
     * @see DynSequencer::has
     */
    inline bool hasSequences(string const &id) {return rmSequencer.has(id);}
    /**
     * @brief Remove all dynamic sequences of rigid motions composing the
     *  dynamic sequencer
     * @see DynSequencer::clear
     */
    inline void clearSequences() {rmSequencer.clear();}
};