#ifndef _GROVE_TREE_WRAPPER_H_
#define _GROVE_TREE_WRAPPER_H_

template <typename Tree> class StaticGrove;

#include <memory>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Wrapper for trees belonging to a StaticGrove so for-each loop can be
 *  used
 * @tparam Tree The type of tree to be wrapped
 */
template <typename Tree>
class GroveTreeWrapper{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The grove where the tree belongs to
     */
    StaticGrove<Tree> &grove;
    /**
     * @brief Index of the tree inside the grove
     */
    size_t index;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Build a wrapper for the tree at given index
     * @param grove Grove containing the tree
     * @param index Index of the tree inside the grove context
     */
    GroveTreeWrapper(StaticGrove<Tree> &grove, size_t const index);
    virtual ~GroveTreeWrapper() = default;

    // ***  FOR-EACH LOOP OPERATORS  *** //
    // ********************************* //
    /**
     * @brief Incremental operator to handle forward iterations in for-each
     *  loops
     */
    GroveTreeWrapper operator++();
    /**
     * @brief Comparison operator to handle last iteration detection in
     *  for-each loops
     */
    bool operator!=(GroveTreeWrapper const &b) const;
    /**
     * @brief Operator to obtain the tree corresponding to current for-each
     *  loop iteration
     */
    const std::shared_ptr<Tree> operator*() const;

};


#include <GroveTreeWrapper.tpp>
#endif
