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
class DynSequentiableMovingObject : public DynMovingObject{
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize a dynamic sequentiable moving object o a stream of
     *  bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the dynamic sequentiable moving
     *  object
     */
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version){
        boost::serialization::void_cast_register<
            DynSequentiableMovingObject,
            DynMovingObject
        >();
        ar &boost::serialization::base_object<
            DynMovingObject
        >(*this);
        ar &dmSequencer;
    }

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
    DynSequentiableMovingObject(
        ScenePart const &sp,
        bool const shallowPrimitives=false
    ) : DynMovingObject(sp, shallowPrimitives) {}
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
    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Add the dynamic sequence of dynamic motions to the dynamic motion
     *  sequencer
     * @param dmSequence Dynamic motion sequence to be added
     * @see DynSequencer::add
     */
    inline void addSequence(shared_ptr<DynSequence<DynMotion>> dmSequence)
    {return dmSequencer.add(dmSequence);}
    /**
     * @brief Remove the dynamic sequence of dynamic motions with given
     *  identifier
     * @param id Identifier of dynamic sequence of dynamic motions to be
     *  removed
     * @see DynSequencer::remove
     */
    inline void removeSequence(string const & id)
    {return dmSequencer.remove(id);}
    /**
     * @brief Get the dynamic sequence of dynamic motions with given identifier
     * @param id Identifier of dynamic sequence of dynamic motions to be
     *  retrieved
     * @return Requested dynamic sequence of dynamic motions if any, nullptr
     *  otherwise
     * @see DynSequencer::get
     */
    inline shared_ptr<DynSequence<DynMotion>> getSequence(string const &id)
    {return dmSequencer.get(id);}
    /**
     * @brief Check if the dynamic sequencer has a dynamic sequence of dynamic
     *  motions with given identifier
     * @param id Identifier of the dynamic sequence of dynamic motions to be
     *  checked
     * @return True if there is a dynamic sequence of dynamic motions with
     *  given identifier, false otherwise
     * @see DynSequencer::has
     */
    inline bool hasSequences(string const &id) {return dmSequencer.has(id);}
    /**
     * @brief Remove all dynamic sequences of dynamic motions composing the
     *  dynamic sequencer
     * @see DynSequencer::clear
     */
    inline void clearSequences() {dmSequencer.clear();}
};