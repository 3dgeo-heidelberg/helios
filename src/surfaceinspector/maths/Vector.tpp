#include <cmath>
#include <vector>

// ***  BASIC VECTOR OPERATIONS  *** //
// ********************************* //
template<typename T>
std::vector<T>
SurfaceInspector::maths::Vector<T>::add(std::vector<T> const u,
                                        std::vector<T> const v)
{
  std::size_t n = u.size();
  std::vector<T> w(n);
  for (std::size_t i = 0; i < n; ++i)
    w[i] = u[i] + v[i];
  return w;
}

template<typename T>
std::vector<T>
SurfaceInspector::maths::Vector<T>::scalarAdd(std::vector<T> const u,
                                              T const scalar)
{
  std::size_t n = u.size();
  std::vector<T> w;
  for (std::size_t i = 0; i < n; i++) {
    w.push_back(u[i] + scalar);
  }
  return w;
}
template<typename T>
std::vector<T>
SurfaceInspector::maths::Vector<T>::scalarSubtract(std::vector<T> const u,
                                                   T const scalar)
{
  std::size_t n = u.size();
  std::vector<T> w;
  for (std::size_t i = 0; i < n; i++) {
    w.push_back(u[i] - scalar);
  }
  return w;
}
template<typename T>
std::vector<T>
SurfaceInspector::maths::Vector<T>::scalarMultiply(std::vector<T> const u,
                                                   T const scalar)
{
  std::size_t n = u.size();
  std::vector<T> w;
  for (std::size_t i = 0; i < n; i++) {
    w.push_back(u[i] * scalar);
  }
  return w;
}
template<typename T>
std::vector<T>
SurfaceInspector::maths::Vector<T>::scalarDivide(std::vector<T> const u,
                                                 T const scalar)
{
  std::size_t n = u.size();
  std::vector<T> w;
  for (std::size_t i = 0; i < n; i++) {
    w.push_back(u[i] / scalar);
  }
  return w;
}

template<typename T>
void
SurfaceInspector::maths::Vector<T>::scalarAddInPlace(std::vector<T>& u,
                                                     T const scalar)
{
  std::size_t n = u.size();
  for (std::size_t i = 0; i < n; i++) {
    u[i] = u[i] + scalar;
  }
}
template<typename T>
void
SurfaceInspector::maths::Vector<T>::scalarSubtractInPlace(std::vector<T>& u,
                                                          T const scalar)
{
  std::size_t n = u.size();
  for (std::size_t i = 0; i < n; i++) {
    u[i] = u[i] - scalar;
  }
}
template<typename T>
void
SurfaceInspector::maths::Vector<T>::scalarMultiplyInPlace(std::vector<T>& u,
                                                          T const scalar)
{
  std::size_t n = u.size();
  for (std::size_t i = 0; i < n; i++) {
    u[i] = u[i] * scalar;
  }
}
template<typename T>
void
SurfaceInspector::maths::Vector<T>::scalarDivideInPlace(std::vector<T>& u,
                                                        T const scalar)
{
  std::size_t n = u.size();
  for (std::size_t i = 0; i < n; i++) {
    u[i] = u[i] / scalar;
  }
}

template<typename T>
std::vector<T>
SurfaceInspector::maths::Vector<T>::negate(std::vector<T> const u)
{
  std::vector<T> v = u;
  std::size_t n = v.size();
  for (std::size_t i = 0; i < n; ++i)
    v[i] = -v[i];
  return v;
}

template<typename T>
bool
SurfaceInspector::maths::Vector<T>::equals(std::vector<T> const u,
                                           std::vector<T> const v)
{
  std::size_t n = u.size();
  if (v.size() != n)
    return false;
  for (std::size_t i = 0; i < n; ++i)
    if (u[i] != v[i])
      return false;
  return true;
}

template<typename T>
T
SurfaceInspector::maths::Vector<T>::min(std::vector<T> const& u)
{
  T uMin = u[0];
  std::size_t n = u.size();
  for (std::size_t i = 1; i < n; i++) {
    if (u[i] < uMin)
      uMin = u[i];
  }
  return uMin;
}

template<typename T>
T
SurfaceInspector::maths::Vector<T>::max(std::vector<T> const& u)
{
  T uMax = u[0];
  std::size_t n = u.size();
  for (std::size_t i = 1; i < n; i++) {
    if (u[i] > uMax)
      uMax = u[i];
  }
  return uMax;
}

template<typename T>
std::size_t
SurfaceInspector::maths::Vector<T>::argmin(std::vector<T> const& u)
{
  std::size_t idx = 0;
  T uMin = u[idx];
  std::size_t n = u.size();
  for (std::size_t i = 1; i < n; i++) {
    if (u[i] < uMin) {
      idx = i;
      uMin = u[idx];
    }
  }
  return idx;
}

template<typename T>
std::size_t
SurfaceInspector::maths::Vector<T>::argmax(std::vector<T> const& u)
{
  std::size_t idx = 0;
  T uMax = u[idx];
  std::size_t n = u.size();
  for (std::size_t i = 1; i < n; i++) {
    if (u[i] > uMax) {
      idx = i;
      uMax = u[idx];
    }
  }
  return idx;
}

template<typename T>
T
SurfaceInspector::maths::Vector<T>::midrange(std::vector<T> const& u)
{
  return (SurfaceInspector::maths::Vector<T>::min(u) +
          SurfaceInspector::maths::Vector<T>::max(u)) /
         2.0;
}

template<typename T>
T
SurfaceInspector::maths::Vector<T>::dotProduct(std::vector<T> const u,
                                               std::vector<T> const v)
{
  T dot = 0.0;
  std::size_t n = u.size();
  for (std::size_t i = 0; i < n; i++) {
    dot += u[i] * v[i];
  }
  return dot;
}

template<typename T>
std::vector<T>
SurfaceInspector::maths::Vector<T>::crossProduct3D(std::vector<T> const u,
                                                   std::vector<T> const v)
{
  return std::vector<T>({ u[1] * v[2] - u[2] * v[1],
                          u[2] * v[0] - u[0] * v[2],
                          u[0] * v[1] - u[1] * v[0] });
}

template<typename T>
std::vector<T>
SurfaceInspector::maths::Vector<T>::hadamardProduct(std::vector<T> const u,
                                                    std::vector<T> const v)
{
  std::vector<T> w;
  std::size_t n = u.size();
  for (std::size_t i = 0; i < n; i++) {
    w.push_back(u[i] * v[i]);
  }
  return w;
}

template<typename T>
T
SurfaceInspector::maths::Vector<T>::squareNorm(std::vector<T> const u)
{
  T norm = 0;
  std::size_t n = u.size();
  for (std::size_t i = 0; i < n; i++) {
    norm += u[i] * u[i];
  }
  return norm;
}

template<typename T>
T
SurfaceInspector::maths::Vector<T>::norm(std::vector<T> const u)
{
  return std::sqrt(squareNorm(u));
}

template<typename T>
std::vector<T>
SurfaceInspector::maths::Vector<T>::normalize(std::vector<T> const u)
{
  return normalize(u, norm(u));
}

template<typename T>
std::vector<T>
SurfaceInspector::maths::Vector<T>::normalize(std::vector<T> const u,
                                              T const norm)
{
  return scalarDivide(u, norm);
}

template<typename T>
void
SurfaceInspector::maths::Vector<T>::normalizeInPlace(std::vector<T>& u)
{
  return normalizeInPlace(u, norm(u));
}

template<typename T>
void
SurfaceInspector::maths::Vector<T>::normalizeInPlace(std::vector<T>& u,
                                                     T const norm)
{
  scalarDivideInPlace(u, norm);
}

template<typename T>
T
SurfaceInspector::maths::Vector<T>::angle(std::vector<T> const u,
                                          std::vector<T> const v,
                                          bool alreadyNormalized)
{
  if (alreadyNormalized)
    return std::acos(dotProduct(u, v));
  return std::acos(dotProduct(u, v) / norm(u) / norm(v));
}

template<typename T>
T
SurfaceInspector::maths::Vector<T>::acuteAngle(std::vector<T> const u,
                                               std::vector<T> const v,
                                               bool alreadyNormalized)
{
  T theta;
  T dot;
  if (alreadyNormalized)
    dot = dotProduct(u, v);
  else
    dot = dotProduct(u, v) / norm(u) / norm(v);
  if (dot > 1.0)
    dot = 1.0;
  if (dot < -1.0)
    dot = -1.0;
  theta = std::acos(dot);
  if (theta > M_PI_2)
    return M_PI - theta;
  return theta;
}

template<typename T>
T
SurfaceInspector::maths::Vector<T>::toAngle2D(std::vector<T> const u)
{
  return atan2(u[1], u[0]);
}

template<typename T>
std::vector<T>
SurfaceInspector::maths::Vector<T>::findOrthogonal(std::vector<T> const v)
{
  // Vector dimensionality
  std::size_t n = v.size();

  // Find non-zero index for v
  std::size_t i = 0;
  if (v[0] == 0.0) {
    for (std::size_t j = 1; j < n; ++j) {
      if (v[j] != 0.0) {
        i = j;
        break;
      }
    }
    if (i == 0) {
      // Exception, v is a null vector
      throw SurfaceInspector::util::SurfaceInspectorException(
        "Exception at Vector::findOrthogonal(vector<T> const) : "
        "cannot handle null vectors");
    }
  }

  // Solve orthogonal vector
  std::vector<T> u(n);
  for (std::size_t j = 0; j < n; ++j)
    u[j] = 1.0;
  u[i] = 0.0;
  for (std::size_t j = 0; j < n; ++j) {
    if (j == i)
      continue;
    u[i] += -v[j];
  }
  u[i] /= v[i];

  // Return
  return u;
}

template<typename T>
std::vector<std::vector<T>>
SurfaceInspector::maths::Vector<T>::xyRotations(size_t const depth,
                                                bool complement)
{
  // Prepare
  T step = M_PI / ((T)depth);
  std::vector<std::vector<T>> rots(depth);

  // Compute semi-circumference space rotations
  for (std::size_t i = 0; i < depth; ++i) {
    T theta = i * step;
    rots[i] = std::vector<T>(2);
    rots[i][0] = std::cos(theta);
    rots[i][1] = std::sin(theta);
  }

  // Compute circumference space rotations through negation
  if (complement) {
    for (std::size_t i = 0; i < depth; ++i) {
      rots.push_back(negate(rots[i]));
    }
  }

  // Return
  return rots;
}

template<typename T>
std::vector<std::vector<T>>
SurfaceInspector::maths::Vector<T>::findOrthonormals(
  std::vector<std::vector<T>> const V)
{
  std::vector<std::vector<T>> orthonormals;
  for (std::vector<T> const& v : V) {
    orthonormals.push_back(findOrthonormal(v));
  }
  return orthonormals;
}
template<typename T>
bool
SurfaceInspector::maths::Vector<T>::isNull(std::vector<T> const& v)
{
  std::size_t n = v.size();
  for (std::size_t i = 0; i < n; ++i) {
    if (v[i] != 0.0 && v[i] != -0.0)
      return false;
  }
  return true;
}

template<typename T>
std::vector<T>
SurfaceInspector::maths::Vector<T>::project(std::vector<T> const v,
                                            std::vector<T> const u)
{
  return scalarMultiply(u, dotProduct(v, u) / dotProduct(u, u));
}

template<typename T>
std::vector<T>
SurfaceInspector::maths::Vector<T>::project(std::vector<T> const v,
                                            std::vector<std::vector<T>> const u)
{
  std::vector<T> proj = project(v, u[0]);
  std::size_t n = u.size();
  for (std::size_t i = 1; i < n; ++i)
    proj = add(proj, project(v, u[i]));
  return proj;
}

template<typename T>
std::vector<T>
SurfaceInspector::maths::Vector<T>::diff(std::vector<T> const u)
{
  std::size_t const n = u.size();
  std::size_t const n_1 = n - 1;
  std::vector<T> v(n_1);
  for (std::size_t i = 0; i < n_1; ++i)
    v[i] = u[i + 1] - u[i];
  return v;
}
