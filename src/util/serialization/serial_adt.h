#pragma once

#include <IterableTreeNode.h>

namespace boost{ namespace serialization{

// ***  ITERABLE TREE NODE SERIALIZATION  *** //
// ****************************************** //
template<class Archive, typename NodeType>
void save_construct_data(
    Archive &ar,
    IterableTreeNode<NodeType> const *itn,
    unsigned int const version
){
    // Save data required to construct instance
    ar << itn->node;
    ar << itn->depth;
}

template<class Archive, typename NodeType>
void load_construct_data(
    Archive &ar,
    IterableTreeNode<NodeType> *itn,
    unsigned int const version
){
    // Load data from archive required to construct new instance
    NodeType *node;
    int depth;
    node << ar;
    depth << ar;

    // Invoke inplace constructor
    ::new(itn)IterableTreeNode<NodeType>(node, depth);
}

}};