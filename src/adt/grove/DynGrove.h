#pragma once

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief A DynGrove is an interface which declares observer-like
 *  functionality so dynamic subjects can notify to the grove their updates,
 *  which acts like a proxy for observer trees.
 *
 * @tparam Subject The type of subject for the observer-like behavior of the
 *  DynGrove interface
 */
template <typename Subject>
class DynGrove{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    virtual ~DynGrove() = default;

    // ***  OBSERVER METHODS  *** //
    // ************************** //
    /**
     * @brief The update method. It is expected that always that a subject
     *  must notify that it has been updated, it calls this method so the
     *  observer can act as corresponds.
     * @param s The subject notifying its update
     */
    virtual void update(Subject &s) = 0;
};