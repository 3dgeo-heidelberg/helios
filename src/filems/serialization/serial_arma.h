#pragma once

#include <armadillo>

namespace boost {
namespace serialization {

/**
 * @brief Split serialization into save and load for armadillo column vectors
 */
template<class Archive>
inline void
serialize(Archive& ar, arma::colvec& vec, const unsigned int version)
{
  boost::serialization::split_free(ar, vec, version);
}

/**
 * @brief Serialize a column vector from armadillo library to a stream of bytes
 * @tparam Archive Type of rendering
 * @param ar Specific rendering for the stream of bytes
 * @param vec Armadillo column vector
 * @param version Version number for the armadillo column vector
 */
template<class Archive>
void
save(Archive& ar, arma::colvec const& vec, const unsigned int version)
{
  size_t const m = vec.size();
  ar & m;
  for (size_t i = 0; i < m; ++i)
    ar& vec[i];
}

/**
 * @brief Deserialize an armadillo column vector from a string of bytes
 * @tparam Archive Type of rendering
 * @param ar Specific rendering for the stream of bytes
 * @param vec Armadillo column vector
 * @param version Version number for the armadillo column vector
 */
template<class Archive>
void
load(Archive& ar, arma::colvec& vec, const unsigned int version)
{
  size_t m;
  ar & m;
  vec.resize(m);
  for (size_t i = 0; i < m; ++i)
    ar& vec[i];
}

/**
 * @brief Split serialization into save and load for armadillo matrix
 */
template<class Archive>
inline void
serialize(Archive& ar, arma::mat& mat, const unsigned int version)
{
  boost::serialization::split_free(ar, mat, version);
}

/**
 * @brief Serialize a matrix from armadillo library to a stream of bytes
 * @tparam Archive Type of rendering
 * @param ar Specific rendering for the stream of bytes
 * @param mat Armadillo matrix
 * @param version Version number for the armadillo matrix
 */
template<class Archive>
void
save(Archive& ar, arma::mat const& mat, const unsigned int version)
{
  size_t const m = mat.n_rows;
  size_t const n = mat.n_cols;
  ar & m & n;
  for (size_t i = 0; i < m; ++i) {   // i-th row
    for (size_t j = 0; j < n; ++j) { // j-th row
      ar & mat.at(i, j);
    }
  }
}

/**
 * @brief Deserialize an armadillo matrix from a string of bytes
 * @tparam Archive Type of rendering
 * @param ar Specific rendering for the stream of bytes
 * @param mat Armadillo matrix
 * @param version Version number for the armadillo matrix
 */
template<class Archive>
void
load(Archive& ar, arma::mat& mat, const unsigned int version)
{
  size_t m, n; // m-rows, n-cols
  ar & m & n;
  mat.resize(m, n);
  for (size_t i = 0; i < m; ++i) {
    for (size_t j = 0; j < n; ++j) {
      ar & mat.at(i, j);
    }
  }
}

}
}
