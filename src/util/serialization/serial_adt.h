#pragma once

#include <serial_adt_utils.h>
#include <IterableTreeNode.h>
#include <MultiThreadKDTreeFactory.h>
#include <MultiThreadSAHKDTreeFactory.h>


namespace boost{ namespace serialization{

// ***  ITERABLE TREE NODE SERIALIZATION  *** //
// ****************************************** //
template <class Archive, typename NodeType>
void save_construct_data(
    Archive &ar,
    IterableTreeNode<NodeType> const *itn,
    unsigned int const version
){
    // Save data required to construct instance
    ar << itn->node;
    ar << itn->depth;
}

template <class Archive, typename NodeType>
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

// ***  KDTREE FACTORY SERIALIZATION  *** //
// ************************************** //

template <class Archive>
void save_construct_data(
    Archive &ar,
    MultiThreadKDTreeFactory const *mtkdtf,
    unsigned int const version
){
    // Save data required to construct instance
    ar << mtkdtf->getKdtf();
    ar << mtkdtf->getNumJobs();
    ar << mtkdtf->getGeomJobs();

    // Save geometric strategy type
    char gsType = KDTREE_FACTORY_EXTRACT_GSTYPE(mtkdtf);
    ar << gsType;
}

template <class Archive>
void load_construct_data(
    Archive &ar,
    MultiThreadKDTreeFactory *mtkdtf,
    unsigned int const version
){
    // Load data from archive required to construct new instance
    std::shared_ptr<SimpleKDTreeFactory> kdtf;
    size_t numJobs, geomJobs;
    char gsType;
    ar >> kdtf;
    ar >> numJobs;
    ar >> geomJobs;
    ar >> gsType;

    // Build geometric strategy and invoke factory's inplace constructor
    KDTREE_FACTORY_INPLACE_CONSTRUCT<MultiThreadKDTreeFactory>(
        mtkdtf,
        kdtf,
        numJobs,
        geomJobs,
        gsType
    );
}

template <class Archive>
void save_construct_data(
    Archive &ar,
    MultiThreadSAHKDTreeFactory const *mtkdtf,
    unsigned int const version
){
    // Save data required to construct instance
    ar << mtkdtf->getKdtf();
    ar << mtkdtf->getNumJobs();
    ar << mtkdtf->getGeomJobs();

    // Save geometric strategy type
    char gsType = KDTREE_FACTORY_EXTRACT_GSTYPE(mtkdtf);
    ar << gsType;
}

template <class Archive>
void load_construct_data(
    Archive &ar,
    MultiThreadSAHKDTreeFactory *mtkdtf,
    unsigned int const version
){
    // Load data from archive required to construct new instance
    std::shared_ptr<SimpleKDTreeFactory> kdtf;
    size_t numJobs, geomJobs;
    char gsType;
    ar >> kdtf;
    ar >> numJobs;
    ar >> geomJobs;
    ar >> gsType;

    // Build geometric strategy and invoke factory's inplace constructor
    KDTREE_FACTORY_INPLACE_CONSTRUCT<MultiThreadSAHKDTreeFactory>(
        mtkdtf,
        kdtf,
        numJobs,
        geomJobs,
        gsType
    );
}

}};