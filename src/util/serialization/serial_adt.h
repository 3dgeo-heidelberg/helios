#pragma once

#include <IterableTreeNode.h>
#include <MultiThreadKDTreeFactory.h>
#include <MultiThreadSAHKDTreeFactory.h>
#include <SimpleKDTreeGeometricStrategy.h>
#include <SAHKDTreeGeometricStrategy.h>
#include <AxisSAHKDTreeGeometricStrategy.h>
#include <FastSAHKDTreeGeometricStrategy.h>

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
    char gsType = 1; // By default : SimpleKDTreeGeometricStratey (1)
    if(mtkdtf->getGS() == nullptr){ // No geometric strategy : 0
        gsType = 0;
    }
    else if(
        dynamic_pointer_cast<FastSAHKDTreeGeometricStrategy>(mtkdtf->getGS())
        != nullptr
    ){ // Fast SAH KDTree geometric strategy : 4
        gsType = 4;
    }
    else if(
        dynamic_pointer_cast<AxisSAHKDTreeGeometricStrategy>(mtkdtf->getGS())
        != nullptr
    ){ // Axis SAH KDTree geometric strategy : 3
        gsType = 3;
    }
    else if(
        dynamic_pointer_cast<SAHKDTreeGeometricStrategy>(mtkdtf->getGS())
        != nullptr
    ){ // SAH KDTree geometric strategy : 2
        gsType = 2;
    }
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
    if(gsType == 1){
        std::shared_ptr<SimpleKDTreeGeometricStrategy> gs = \
        std::make_shared<SimpleKDTreeGeometricStrategy>(*kdtf);
        ::new(mtkdtf)MultiThreadSAHKDTreeFactory(
            kdtf, gs, numJobs, geomJobs
        );
    }
    else if(gsType == 2){
        std::shared_ptr<SAHKDTreeGeometricStrategy> gs = \
        std::make_shared<SAHKDTreeGeometricStrategy>(
            *std::dynamic_pointer_cast<SAHKDTreeFactory>(kdtf)
        );
        ::new(mtkdtf)MultiThreadSAHKDTreeFactory(
            kdtf, gs, numJobs, geomJobs
        );
    }
    else if(gsType == 3){
        std::shared_ptr<AxisSAHKDTreeGeometricStrategy> gs = \
        std::make_shared<AxisSAHKDTreeGeometricStrategy>(
            *std::dynamic_pointer_cast<AxisSAHKDTreeFactory>(kdtf)
        );
        ::new(mtkdtf)MultiThreadSAHKDTreeFactory(
            kdtf, gs, numJobs, geomJobs
        );
    }
    else if(gsType == 4){
        std::shared_ptr<FastSAHKDTreeGeometricStrategy> gs = \
        std::make_shared<FastSAHKDTreeGeometricStrategy>(
            *std::dynamic_pointer_cast<FastSAHKDTreeFactory>(kdtf)
        );
        ::new(mtkdtf)MultiThreadSAHKDTreeFactory(
            kdtf, gs, numJobs, geomJobs
        );
    }
    else{
        ::new(mtkdtf)MultiThreadSAHKDTreeFactory(
            kdtf, nullptr, numJobs, geomJobs
        );
    }
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
    char gsType = 1; // By default : SimpleKDTreeGeometricStratey (1)
    if(mtkdtf->getGS() == nullptr){ // No geometric strategy : 0
        gsType = 0;
    }
    else if(
        dynamic_pointer_cast<FastSAHKDTreeGeometricStrategy>(mtkdtf->getGS())
        != nullptr
    ){ // Fast SAH KDTree geometric strategy : 4
        gsType = 4;
    }
    else if(
        dynamic_pointer_cast<AxisSAHKDTreeGeometricStrategy>(mtkdtf->getGS())
        != nullptr
    ){ // Axis SAH KDTree geometric strategy : 3
        gsType = 3;
    }
    else if(
        dynamic_pointer_cast<SAHKDTreeGeometricStrategy>(mtkdtf->getGS())
        != nullptr
    ){ // SAH KDTree geometric strategy : 2
        gsType = 2;
    }
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
    if(gsType == 1){
        std::shared_ptr<SimpleKDTreeGeometricStrategy> gs = \
            std::make_shared<SimpleKDTreeGeometricStrategy>(*kdtf);
        ::new(mtkdtf)MultiThreadSAHKDTreeFactory(
            kdtf, gs, numJobs, geomJobs
        );
    }
    else if(gsType == 2){
        std::shared_ptr<SAHKDTreeGeometricStrategy> gs = \
        std::make_shared<SAHKDTreeGeometricStrategy>(
            *std::dynamic_pointer_cast<SAHKDTreeFactory>(kdtf)
        );
        ::new(mtkdtf)MultiThreadSAHKDTreeFactory(
            kdtf, gs, numJobs, geomJobs
        );
    }
    else if(gsType == 3){
        std::shared_ptr<AxisSAHKDTreeGeometricStrategy> gs = \
        std::make_shared<AxisSAHKDTreeGeometricStrategy>(
            *std::dynamic_pointer_cast<AxisSAHKDTreeFactory>(kdtf)
        );
        ::new(mtkdtf)MultiThreadSAHKDTreeFactory(
            kdtf, gs, numJobs, geomJobs
        );
    }
    else if(gsType == 4){
        std::shared_ptr<FastSAHKDTreeGeometricStrategy> gs = \
        std::make_shared<FastSAHKDTreeGeometricStrategy>(
            *std::dynamic_pointer_cast<FastSAHKDTreeFactory>(kdtf)
        );
        ::new(mtkdtf)MultiThreadSAHKDTreeFactory(
            kdtf, gs, numJobs, geomJobs
        );
    }
    else{
        ::new(mtkdtf)MultiThreadSAHKDTreeFactory(
            kdtf, nullptr, numJobs, geomJobs
        );
    }
}

/*
 * TODO Rethink : Redesign save/load of multithread factories to avoid
 * redundancies and simplify design
 */

}};