#ifndef _BASIC_DYN_GROVE_H_
#define _BASIC_DYN_GROVE_H_

#include <BasicStaticGrove.h>
#include <DynGrove.h>

#include <memory>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Basic implementation of a DynGrove which extends BasicStaticGrove
 *  to provided dynamic funcionalities
 *
 * @tparam Tree Tyhe type of the tree to be handled
 * @see BasicStaticGrove
 * @see DynGrove
 */
template <typename Tree, typename Subject, typename SubjectId>
class BasicDynGrove : public BasicStaticGrove<Tree>, DynGrove<Subject> {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Map associating each subject with its corresponding tree.
     *
     * It is a one-to-one relationship, so each subject can be mapped to at
     *  most one tree.
     */
    std::unordered_map<SubjectId, std::shared_ptr<Tree>> observersMap;

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

};

#include <BasicDynGrove.tpp>
#endif