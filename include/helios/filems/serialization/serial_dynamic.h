#pragma once

#include <helios/scene/dynamic/DynMotion.h>

namespace boost {
namespace serialization {

// ***  DynMotion SERIALIZATION  *** //
// ********************************* //
/**
 * @brief Handle dynamic motion save operation with no default constructor
 */
template<class Archive>
void
save_construct_data(Archive& ar,
                    const DynMotion* dm,
                    const unsigned int version)
{
  // Write what is needed to construct
  ar << dm->getC();
  ar << dm->getA();
}
/**
 * @brief Handle dynamic motion load operation with no default constructor
 */
template<class Archive>
void
load_construct_data(Archive& ar, DynMotion* dm, const unsigned int version)
{
  // Construct from reading what is needed to construct
  arma::colvec C;
  ar >> C;
  arma::mat A;
  ar >> A;
  ::new (dm) DynMotion(C, A);
}

}
}
