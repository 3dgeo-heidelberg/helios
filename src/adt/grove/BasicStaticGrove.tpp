#include <BasicStaticGrove.h>

using std::shared_ptr;

// ***  QUERY METHODS  *** //
// *********************** //
template <typename Tree>
bool BasicStaticGrove<Tree>::hasTrees() const{
    return !trees.empty();
}
template <typename Tree>
size_t BasicStaticGrove<Tree>::getNumTrees() const{
    return trees.size();
}

// ***  MANIPULATION METHODS  *** //
// ****************************** //
template <typename Tree>
void BasicStaticGrove<Tree>::addTree(std::shared_ptr<Tree> tree){
    trees.push_back(tree);
}
template <typename Tree>
void BasicStaticGrove<Tree>::removeTree(size_t const index){
    trees.erase(trees.begin() + index);
}
template <typename Tree>
void BasicStaticGrove<Tree>::removeTrees(
    size_t const startIndex, size_t const endIndex
){
    trees.erase(trees.begin() + startIndex, trees.begin() + endIndex);
}
template <typename Tree>
void BasicStaticGrove<Tree>::removeAll(){
    trees.clear();
}

// ***  FOR LOOP METHODS  *** //
// ************************** //
template <typename Tree>
Tree & BasicStaticGrove<Tree>::getTreeReference(
    size_t const index
) const{
    return *trees[index];
}
template <typename Tree>
shared_ptr<Tree> BasicStaticGrove<Tree>::getTreeShared(
    size_t const index
) const{
    return trees[index];
}
template <typename Tree>
Tree * BasicStaticGrove<Tree>::getTreePointer(size_t const index) const{
    return trees[index].get();
}
template <typename Tree>
void BasicStaticGrove<Tree>::replaceTree(
    size_t const index,
    std::shared_ptr<Tree> tree
){
    trees[index] = tree;
}

// ***  WHILE LOOP METHODS  *** //
// **************************** //
template <typename Tree>
void BasicStaticGrove<Tree>::toZeroTree() {
    whileIter = 0;
}
template <typename Tree>
bool BasicStaticGrove<Tree>::hasNextTree() const{
    return getNumTrees() > whileIter;
}
template <typename Tree>
Tree & BasicStaticGrove<Tree>::nextTreeReference() {
    if(whileIter == getNumTrees()) whileIter = 0;
    Tree &tree = getTreeReference(whileIter);
    ++whileIter;
    return tree;
}
template <typename Tree>
shared_ptr<Tree> BasicStaticGrove<Tree>::nextTreeShared() {
    if(whileIter == getNumTrees()) whileIter = 0;
    shared_ptr<Tree> tree = getTreeShared(whileIter);
    ++whileIter;
    return tree;
}
template <typename Tree>
Tree * BasicStaticGrove<Tree>::nextTreePointer() {
    if(whileIter == getNumTrees()) whileIter = 0;
    Tree *tree = getTreePointer(whileIter);
    ++whileIter;
    return tree;
}
