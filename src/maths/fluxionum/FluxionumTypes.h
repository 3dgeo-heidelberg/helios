#pragma once

namespace fluxionum {

// ***  ENUMERATIONS  *** //
// ********************** //
/**
 * @brief Potential differential types of any fluxionum::DiffDesignMatrix
 * @see fluxionum::DiffDesignMatrix
 */
enum class DiffDesignMatrixType
{
  FORWARD_FINITE_DIFFERENCES,
  CENTRAL_FINITE_DIFFERENCES,
  UNKNOWN
};

}
