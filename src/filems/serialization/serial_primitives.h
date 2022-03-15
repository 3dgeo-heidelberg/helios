#pragma once

#include <Triangle.h>

namespace boost{namespace serialization{

// ***  TRIANGLE  SERIALIZATION  *** //
// ********************************* //
/**
 * @brief Handle triangle primitive save operation with no default constructor
 */
template<class Archive>
void save_construct_data(
    Archive & ar,
    const Triangle * t,
    const unsigned int file_version
){
    // save data required to construct instance
    ar << t->verts;
}
/**
 * @brief Handle triangle primitive load operation with no default constructor
 */
template<class Archive>
void load_construct_data(
    Archive & ar,
    Triangle * t,
    const unsigned int file_version
){
    // retrieve data from archive required to construct new instance
    Vertex verts[3];
    ar >> verts;
    // invoke inplace constructor to initialize instance of my_class
    ::new(t)Triangle(verts[0], verts[1], verts[2]);
}


}};