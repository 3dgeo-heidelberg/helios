#ifndef _BASIC_DYN_GROVE_H_
#define _BASIC_DYN_GROVE_H_

#include <BasicStaticGrove.h>
#include <DynGrove.h>

#include <memory>
#include <vector>

template <typename Tree, typename Subject>
class BasicDynGroveSubject;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Basic implementation of a DynGrove which extends BasicStaticGrove
 *  to provided dynamic funcionalities
 *
 * @tparam Tree The type of the tree to be handled
 * @tparam Subject The type of the subject that can be observed
 * @see BasicStaticGrove
 * @see DynGrove
 */
template <typename Tree, typename Subject>
class BasicDynGrove : public BasicStaticGrove<Tree>, public DynGrove<Subject> {
protected:
    using BasicStaticGrove<Tree>::trees;
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Vector with pointers to subjects.
     *
     * The typical observer pattern does not require the observer
     *  (BasicDynGrove) to be aware of all subjects, just to handle their
     *  notifications. Nonetheless, to avoid using a map to know to which
     *  sub-observer (Tree) each subjects is associated, this vector is used.
     *  In consequence, all subjects are handled by the BasicDynGrove to
     *  have their indices corresponding with vector indices. This reduces the
     *  computational cost of calculating the key-value for a map always that
     *  a subject notifies to the observer (which happens too frequently
     *  during simulation).
     */
    std::vector<BasicDynGroveSubject<Tree, Subject> *> subjects;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for BasicDynGrove
     */
    BasicDynGrove() : BasicStaticGrove<Tree>() {}
    virtual ~BasicDynGrove() = default;

    // ***  OBSERVER METHODS  *** //
    // ************************** //
    /**
     * @see DynGrove::update
     */
    void update(Subject &s) override;
    /**
     * @brief Add a new subject which identifier is automatically and
     *  internally computed.
     *
     * Notice when adding a subject, its identifier will be updated
     *  through the BasicDynGroveSubject::setGroveSubjectId method
     *
     * @param subject Reference to the subject itself
     * @param tree Tree associated to new subject
     */
    virtual void addSubject(
        BasicDynGroveSubject<Tree, Subject> *subject,
        std::shared_ptr<Tree> tree
    );
    /**
     * @brief Remove subject with given identifier from the grove.
     *
     * Notice when removing a subject, all other subject which
     *  identifier needs to be updated will be automatically updated.
     * (Through BasicDynGroveSubject::setGroveSubjectId method)
     *
     * @param id Identifier of the subject to be removed
     */
    virtual void removeSubject(BasicDynGroveSubject<Tree, Subject> *subject);

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Obtain a read-only reference to vector of subjects
     * @return Read-only reference to vector of subjects
     */
    virtual std::vector<BasicDynGroveSubject<Tree, Subject> *> const&
    getSubjects(){return subjects;}
};

#include <BasicDynGrove.tpp>
#endif