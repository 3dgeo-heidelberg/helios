#ifndef _BASIC_STATIC_GROVE_H_
#define _BASIC_STATIC_GROVE_H_

#include <StaticGrove.h>

#include <vector>
#include <memory>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Basic implementation of a StaticGrove
 *
 * @tparam Tree The type of tree to be handled
 * @see StaticGrove
 */
template <typename Tree>
class BasicStaticGrove : public StaticGrove<Tree> {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The trees contained in the grove
     */
    std::vector<std::shared_ptr<Tree>> trees;
    /**
     * @brief Current iteration for while loop friendly methods
     */
    size_t whileIter;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for BasicStaticGrove
     */
    BasicStaticGrove() = default;
    virtual ~BasicStaticGrove() = default;

    // ***  QUERY METHODS  *** //
    // *********************** //
    /**
     * @see StaticGrove::hasTrees
     */
    bool hasTrees() const override;
    /**
     * @see StaticGrove::getNumTrees
     */
    size_t getNumTrees() const override;

    // ***  MANIPULATION METHODS  *** //
    // ****************************** //
    /**
     * @see StaticGrove::addTree
     */
    void addTree(std::shared_ptr<Tree> tree) override;
    /**
     * @see StaticGrove::removeTree
     */
    void removeTree(size_t const index) override;
    /**
     * @see StaticGrove::removeTrees
     */
    void removeTrees(size_t const startIndex, size_t const endIndex) override;
    /**
     * @see StaticGrove::removeAll
     */
    void removeAll() override;
    void replaceTree(size_t const index, std::shared_ptr<Tree> tree) override;

    // ***  FOR LOOP METHODS  *** //
    // ************************** //
    /**
     * @see StaticGrove::getTreeReference
     */
    Tree & getTreeReference(size_t const index) const override;
    /**
     * @see StaticGrove::getTreeShared
     */
    std::shared_ptr<Tree> getTreeShared(size_t const index) const override;
    /**
     * @see StaticGrove::getTreePointer
     */
    Tree * getTreePointer(size_t const index) const override;

    // ***  WHILE LOOP METHODS  *** //
    // **************************** //
    /**
     * @see StaticGrove::toZeroTree
     */
    void toZeroTree() override;
    /**
     * @see StaticGrove::hasNextTree
     */
    bool hasNextTree() const override;
    /**
     * @see StaticGrove::nextTreeReference
     */
    Tree & nextTreeReference() override;
    /**
     * @see StaticGrove::nextTreeShared
     */
    std::shared_ptr<Tree> nextTreeShared() override;
    /**
     * @see StaticGrove::nextTreePointer
     */
    Tree * nextTreePointer() override;

};

#include <BasicStaticGrove.tpp>
#endif