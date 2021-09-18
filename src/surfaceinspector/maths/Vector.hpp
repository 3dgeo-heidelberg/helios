#ifndef _SURFACEINSPECTOR_MATHS_VECTOR_HPP_
#define _SURFACEINSPECTOR_MATHS_VECTOR_HPP_

#include <vector>

#include <surfaceinspector/util/Object.hpp>
#include <surfaceinspector/util/SurfaceInspectorException.hpp>

using std::vector;

using SurfaceInspector::util::Object;

namespace SurfaceInspector{ namespace maths {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @tparam T Type of element used by the vector
 * @brief Class providing vector operations
 */
template <typename T>
class Vector : public Object{
public:
    // ***  BASIC VECTOR OPERATIONS  *** //
    // ********************************* //
    /**
     * @brief Add two vectors
     * @return Vector from addition
     */
    static vector<T> add(vector<T> const u, vector<T> const v);

    /**
     * @brief Subtract two vectors
     * @return Vector from subtraction
     */
    static inline vector<T> subtract(vector<T> const u, vector<T> const v)
    {return add(u, negate(v));}

    /**
     * @brief Add the scalar to all elements in vector \f$\overline{u}\f$
     * @return Vector resulting from addition
     */
    static vector<T> scalarAdd(vector<T> const u, T const scalar);
    /**
     * @brief Subtract the scalar to all elements in vector \f$\overline{u}\f$
     * @return Vector resulting from subtraction
     */
    static vector<T> scalarSubtract(vector<T> const u, T const scalar);
    /**
     * @brief Multiply all elements in vector \f$overline{u}\f$ by scalar
     * @return Vector resulting from multiplication
     */
    static vector<T> scalarMultiply(vector<T> const u, T const scalar);
    /**
     * @brief Divide all elements in vector \f$\overline{u}\f$ by scalar
     * @return Vector resulting from division
     */
    static vector<T> scalarDivide(vector<T> const u, T const scalar);
    /**
     * @brief Like scalarAdd function but modifying given vector
     * @see Vector::scalarAdd
     */
    static void scalarAddInPlace(vector<T> &u, T const scalar);
    /**
     * @brief Like scalarSubtract function but modifying given vector
     * @see Vector::scalarSubtract
     */
    static void scalarSubtractInPlace(vector<T> &u, T const scalar);
    /**
     * @brief Like scalarMultiply function but modifying given vector
     * @see Vector::scalarMultiply
     */
    static void scalarMultiplyInPlace(vector<T> &u, T const scalar);
    /**
     * @brief Like scalarDivide function but modifying given vector
     * @see Vector::scalarDivide
     */
    static void scalarDivideInPlace(vector<T> &u, T const scalar);
    /**
     * @brief Compute the opposite of given vector \f$u\f$
     *
     * \f[
     *  u = (u_{1}, ..., u_{n}) \implies -u = (-u_{1}, ..., -u_{n})
     * \f]
     * @param u Vector to be negated
     * @return Negated \f$u\f$
     */
    static vector<T> negate(vector<T> const u);
    /**
     * @brief Check if you vectors are equal or not.
     *
     * Two vectors \f$u\f$ and \f$v\f$ are considered to be equal when
     *  \f$\forall i,\, u_{i}=v_{i}\f$
     *
     * @param u First vector to compare with
     * @param v Second vector to compare with
     * @return True if vectors are exactly equal, false otherwise
     */
    static bool equals(vector<T> const u, vector<T> const v);
    /**
     * @brief Obtain the minimum value in given vector
     * @param u Vector which minimum value must be obtained
     * @return Minimum value from given vector
     */
    static T min(vector<T> const &u);
    /**
     * @brief Obtain the maximum value in given vector
     * @param u Vector which maximum value must be obtained
     * @return Maximum value from given vector
     */
    static T max(vector<T> const &u);
    /**
     * @brief Obtain the index of minimum value in given vector
     * @param u Vector which minimum value index must be obtained
     * @return Minimum value index from given vector
     */
    static size_t argmin(vector<T> const &u);
    /**
     * @brief Obtain the index of maximum value in given vector
     * @param u Vector which maximum value index must be obtained
     * @return Maximum value index from given vector
     */
    static size_t argmax(vector<T> const &u);
    /**
     * @brief Compute the midrange \f$\lambda\f$ for given vector
     *
     * \f[
     *  \lambda = \frac{u_{max}+u_{min}}{2}
     * \f]
     *
     * @param u Vector which midrange must be obtained
     * @return Midrange for given vector
     */
    static T midrange(vector<T> const &u);
    /**
     * @brief Compute the dot product between vectors \f$\overline{u}\f$ and
     *  \f$\overline{v}\f$
     *
     * \f[
     *  \overline{u} \cdot \overline{v} =
     *  \sum_{i=1}^{n}{u_{i} v_{i}}
     * \f]
     *
     * @param u First vector
     * @param v Second vector
     * @return Dot product between \f$\overline{u}\f$ and \f$\overline{v}\f$
     */
    static T dotProduct(vector<T> const u, vector <T> const v);
    /**
     * @brief Compute the 3D cross product between vectors \f$\overline{u}\f$
     *  and \f$\overline{v}\f$
     *
     * \f[
     *  \overline{w} = \overline{u} \times \overline{v} =
     *  (
     *  u_{y}v_{z} - u_{z}v_{y},
     *  u_{z}v_{x} - u_{x}v_{z},
     *  u_{x}v_{y} - u_{y}v_{x}
     *  )
     * \f]
     *
     * @param u First vector
     * @param v Second vector
     * @return Cross product between \f$\overline{u}\f$ and \f$\overline{v}\f$
     */
    static vector<T> crossProduct3D(vector<T> const u, vector<T> const v);
    /**
     * @brief Compute the hadamard product between vectors \f$\overline{u}\f$
     *  and \f$\overline{v}\f$
     *
     * \f[
     *  \overline{w} = \overline{u} \odot \overline{v} =
     *      (u_{1}v_{1}, \ldots , u_{n}v_{n})
     * \f]
     *
     * @param u First vector
     * @param v Second vector
     * @return Hadamard product between \f$\overline{u}\f$ and
     *  \f$\overline{v}\f$
     */
    static vector<T> hadamardProduct(vector<T> const u, vector<T> const v);
    /**
     * @brief Compute the square norm (magnitude) of vector \f$\overline{u}\f$
     *
     * \f[
     *  ||\overline{u}||^{2} = \sum_{i=1}^{n}{u_{i}^{2}}
     * \f]
     *
     * @param u Vector which square norm (magnitude) must be computed
     * @return Square norm (magnitude) of vector \f$\overline{u}\f$
     */
    static T squareNorm(vector<T> const u);
    /**
     * @see Vector::squareNorm
     */
    static inline T squareMagnitude(vector<T> const u)
    {return normSquare(u);}
    /**
     * @brief Compute the norm (magnitude) of vector \f$\overline{u}\f$, which
     *  is the square root of its square norm
     *
     * \f[
     *  ||\overline{u}|| = \sqrt{\sum_{i=1}^{n}{u_{i}^{2}}}
     * \f]
     *
     * @param u Vector which norm (magnitude) must be computed
     * @return Norm (magnitude) of vector \f$\overline{u}\f$
     * @see Vector::squareNorm
     */
    static T norm(vector<T> const u);
    /**
     * @see Vector::norm
     */
    static inline T magnitude(vector<T> const u)
    {return norm(u);}
    /**
     * @brief Normalize given vector \f$\overline{u}\f$, obtaining
     *  \f$\hat{u}\f$
     *
     * \f[
     *  \hat{u} = \left(\frac{u_{i}}{||\overline{u}||} ,\ldots,
     *      \frac{u_{i}}{||\overline{u}||}\right)
     * \f]
     *
     * @param u Vector to be normalized
     * @return Normalized vector \f$\hat{u}\f$
     */
    static vector<T> normalize(vector<T> const u);
    /**
     * @brief Like Vector::normalize(vector<T> const) but providing a
     *  precomputed norm so it is not necessary to compute it. Hence, it is
     *  faster as it performs less operations
     * @see Vector::normalize(vector<T> const)
     */
    static vector<T> normalize(vector<T> const u, T const norm);
    /**
     * @brief Like Vector::normalize(vector<T> const) but modifying given
     *  vector instead of creating new one
     * @see Vector::normalize(vector<T> const)
     */
    static void normalizeInPlace(vector<T> &u);
    /**
     * @brief Like Vector::normalize(vector<T> const, T const) but modifying
     *  given vector instead of creating new one
     * @see Vector::normalize(vector<T> const, T const)
     */
    static void normalizeInPlace(vector<T> &u, T const norm);
    /**
     * @brief Compute the angle between two given vectors \f$overline{u}\f$
     *  and \f$\overline{v}\f$
     *
     * \f[
     *  \theta = \arccos{\left(
     *      \frac{\overline{u}\cdot\overline{v}}
     *          {||\overline{u}||\cdot||\overline{v}||}
     *  \right)}
     * \f]
     *
     * @param u First vector
     * @param v Second vector
     * @param alreadyNormalized True when vectors are already normalized so
     *  normalization is not necessary and computation will be faster, false
     *  otherwise
     * @return Angle between vectors \f$\overline{u}\f$ and \f$\overline{v}\f$
     */
    static T angle(
        vector<T> const u,
        vector<T> const v,
        bool alreadyNormalized=false
    );
    /**
     * @brief Compute the \f$\theta \leq \frac{\pi}{2}\f$ angle between two given
     *  vectors \f$\vec{u}\f$ and \f$\vec{v}\f$.
     *
     * \f[
     *  {\theta}' = \arccos{\left(
     *      \frac{\langle{\overline{u},\overline{v}}\rangle}
     *          {||\overline{u}||\cdot||\overline{v}||}
     *  \right)}
     * \f]
     *
     * When \f${\theta}' \leq \frac{\pi}{2}\f$ then \f$\theta = {\theta}'\f$,
     *  otherwise \f$\theta = \pi - {\theta}'\f$
     *
     * @param u First vector
     * @param v Second vector
     * @param alreadyNormalized True when vectors are already normalized so
     *  normalization is not necessary and computation will be faster, false
     *  otherwise
     * @return \f$\theta \leq \frac{\pi}{2}\f$ angle between vectors
     *  \f$\overline{u}\f$ and \f$\overline{v}\f$
     */
    static T acuteAngle(
        vector<T> const u,
        vector<T> const v,
        bool alreadyNormalized=false
    );
    /**
     * @brief Compute the 2D angle coming from two first components of
     *  vector \f$\overline{u}\f$
     * @param u Vector which 2D angle must be obtained
     * @return Angle of first two components of \f$\overline{u}\f$ in interval
     *  \f$[-\pi, +\pi]\f$
     */
    static T toAngle2D(vector<T> const u);

    /**
     * @brief Find a orthogonal vector with respect to \f$v\f$
     *
     * Let u be the orthogonal vector which can be defined as follows:
     * \f[
     *  u_{i} = \frac{1}{n_{i}}\sum_{i{\neq}j}{u_{j}v_{j}}
     * \f]
     *
     * @param v Vector which orthogonal must be found
     * @return Vector orthogonal to \f$v\f$
     * @see Vector::findOrthonormal(vector<T> const)
     */
    static vector<T> findOrthogonal(vector<T> const v);

    /**
     * @brief Like findOrthogonal(vector<T> const) but returning a normalized
     *  orthogonal vector
     * @return Orthonormal vector with respect to v
     * @see Vector::findOrthogonal(vector<T> const)
     * @see Vector::normalize
     */
    static vector<T> findOrthonormal(vector<T> const v)
    {return normalize(findOrthogonal(v));}

    /**
     * @brief Like findOrthonormal(vector<T> const) but applied to a vector of
     *  vectors instead of a single vector
     * @see Vector::findOrthonormal(vector<T> const)
     */
    static vector<vector<T>> findOrthonormals(vector<vector<T>> const V);

    /**
     * @brief Compute normal vectors representing different rotations on 2D
     *
     * Rotations are computed according to following formula
     *
     * \f[
     *  x' = cos(\theta)x - sin(\theta)y \\
     *  y' = sin(\theta)x + cos(\theta)y
     * \f]
     *
     * @param depth The number of rotations computed in the semi-circumference
     *  space.
     * @param complement When it is false, as many vectors as depth will be
     *  returned. However, when it is true, as many vectors as depth x 2 will
     *  be returned, since the entire circumference will be considered through
     *  negation of vectors in the semi-circumference space
     * @return Either depth rotations on the semi-circumference space (when
     *  complement is false) or (2 x depth) rotations on the circumference
     *  space (when complement is true)
     */
    static vector<vector<T>> xyRotations(
        size_t const depth,
        bool complement
    );

    /**
     * @brief Check if \f$\vec{v}\f$ is a null vector (all zero) or not
     * @param v Check to be checked for nullity
     * @return True if \f$\vec{v}\f$ is a null vector, false otherwise.
     */
    static bool isNull(vector<T> const &v);

    /**
     * @brief Project vector \f$v\f$ over vector \f$u\f$
     *
     * \f[
     *  \mathrm{proj}_{u}{v} =
     *      \frac{\langle{u,v}\rangle}{\langle{u,u}\rangle} u
     * \f]
     *
     * @param v Vector to be projected
     * @param u Vector to project over it
     * @return Projection of v over u
     */
    static vector<T> project(vector<T> const v, vector<T> const u);

    /**
     * @brief Project vector \f$v\f$ over subspace \f$u\f$
     *
     * Considering \f$u = \langle{b_{1}, \ldots, b_{n}}\rangle\f$
     *
     * \f[
     *  \mathrm{proj}_{u}{v} =
     *      \frac{\langle{v,b_{1}}\rangle}{\langle{b_{1},b_{1}}\rangle} b_{1}
     *      + \ldots +
     *      \frac{\langle{v,b_{n}}\rangle}{\langle{b_{n},b_{n}}\rangle} b_{n}
     *      =
     *      \sum_{i=1}^{n}{\frac{\langle{v,b_{i}}\rangle}{||b_{i}||^{2}}b_{i}}
     * \f]
     *
     * @param v Vector to be projected
     * @param u Basis of the vector subspace to project over it
     * @return Projection of v over u
     */
    static vector<T> project(vector<T> const v, vector<vector<T>> const u);
    /**
     * @brief Compute \f$v\f$ as the discrete differences of vector \f$u\f$
     *
     * The cardinality of
     *  \f$v\f$ will be \f$|v| = |u| - 1\f$.
     *
     * Also, \f$v_i = u_{i+1} - u_{i}\f$, so:
     * \f[
     *  v = \left(u_{2}-u_{1}, \ldots, u_{n}-u_{n-1}\right)
     * \f]
     *
     * @param u Vector to differentiate
     * @return \f$v\f$ vector as the discrete differences of vector \f$u\f$
     */
    static vector<T> diff(vector<T> const u);
};

}}

#include <surfaceinspector/maths/Vector.tpp>

#endif
