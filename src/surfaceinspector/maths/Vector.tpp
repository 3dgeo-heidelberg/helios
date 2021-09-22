#ifndef _SURFACEINSPECTOR_MATHS_VECTOR_HPP_
#include <surfaceinspector/maths/Vector.hpp>
#endif

#include <cmath>

using std::sqrt;
using std::acos;
using std::atan2;

using SurfaceInspector::maths::Vector;
using SurfaceInspector::util::SurfaceInspectorException;

// ***  BASIC VECTOR OPERATIONS  *** //
// ********************************* //
template <typename T>
vector<T> Vector<T>::add(vector<T> const u, vector<T> const v){
    size_t n = u.size();
    vector<T> w(n);
    for(size_t i = 0 ; i < n ; ++i) w[i] = u[i] + v[i];
    return w;
}

template <typename T>
vector<T> Vector<T>::scalarAdd(vector<T> const u, T const scalar){
    size_t n = u.size();
    vector<T> w;
    for(size_t i = 0 ; i < n ; i++){
        w.push_back(u[i] + scalar);
    }
    return w;
}
template <typename T>
vector<T> Vector<T>::scalarSubtract(vector<T> const u, T const scalar){
    size_t n = u.size();
    vector<T> w;
    for(size_t i = 0 ; i < n ; i++){
        w.push_back(u[i] - scalar);
    }
    return w;
}
template <typename T>
vector<T> Vector<T>::scalarMultiply(vector<T> const u, T const scalar){
    size_t n = u.size();
    vector<T> w;
    for(size_t i = 0 ; i < n ; i++){
        w.push_back(u[i] * scalar);
    }
    return w;
}
template <typename T>
vector<T> Vector<T>::scalarDivide(vector<T> const u, T const scalar){
    size_t n = u.size();
    vector<T> w;
    for(size_t i = 0 ; i < n ; i++){
        w.push_back(u[i] / scalar);
    }
    return w;
}

template <typename T>
void Vector<T>::scalarAddInPlace(vector<T> &u, T const scalar){
    size_t n = u.size();
    for(size_t i = 0 ; i < n ; i++){
        u[i] = u[i] + scalar;
    }
}
template <typename T>
void Vector<T>::scalarSubtractInPlace(vector<T> &u, T const scalar){
    size_t n = u.size();
    for(size_t i = 0 ; i < n ; i++){
        u[i] = u[i] - scalar;
    }
}
template <typename T>
void Vector<T>::scalarMultiplyInPlace(vector<T> &u, T const scalar){
    size_t n = u.size();
    for(size_t i = 0 ; i < n ; i++){
        u[i] = u[i] * scalar;
    }
}
template <typename T>
void Vector<T>::scalarDivideInPlace(vector<T> &u, T const scalar){
    size_t n = u.size();
    for(size_t i = 0 ; i < n ; i++){
        u[i] = u[i] / scalar;
    }
}

template <typename T>
vector<T> Vector<T>::negate(vector<T> const u){
    vector<T> v = u;
    size_t n = v.size();
    for(size_t i = 0 ; i < n ; ++i) v[i] = -v[i];
    return v;
}

template <typename T>
bool Vector<T>::equals(vector<T> const u, vector<T> const v){
    size_t n = u.size();
    if(v.size() != n) return false;
    for(size_t i = 0 ; i < n ; ++i) if(u[i] != v[i]) return false;
    return true;
}

template <typename T> T Vector<T>::min(vector<T> const &u){
    T uMin = u[0];
    size_t n = u.size();
    for(size_t i = 1; i < n ; i++){
        if(u[i] < uMin) uMin = u[i];
    }
    return uMin;
}

template <typename T> T Vector<T>::max(vector<T> const &u){
    T uMax = u[0];
    size_t n = u.size();
    for(size_t i = 1 ; i < n ; i++){
        if(u[i] > uMax) uMax = u[i];
    }
    return uMax;
}
template <typename T> size_t Vector<T>::argmin(vector<T> const &u){
    size_t idx = 0;
    T uMin = u[idx];
    size_t n = u.size();
    for(size_t i = 1; i < n ; i++){
        if(u[i] < uMin){
            idx = i;
            uMin = u[idx];
        }
    }
    return idx;
}
template <typename T> size_t Vector<T>::argmax(vector<T> const &u){
    size_t idx = 0;
    T uMax = u[idx];
    size_t n = u.size();
    for(size_t i = 1 ; i < n ; i++){
        if(u[i] > uMax){
            idx = i;
            uMax = u[idx];
        }
    }
    return idx;
}

template <typename T> T Vector<T>::midrange(vector<T> const &u){
    return (Vector<T>::min(u) + Vector<T>::max(u)) / 2.0;
}

template <typename T>
T Vector<T>::dotProduct(vector<T> const u, vector <T> const v){
    T dot = 0.0;
    size_t n = u.size();
    for(size_t i = 0; i < n ; i++){
        dot += u[i]*v[i];
    }
    return dot;
}

template <typename T>
vector<T> Vector<T>::crossProduct3D(vector<T> const u, vector<T> const v){
    return vector<T>({
        u[1]*v[2] - u[2]*v[1],
        u[2]*v[0] - u[0]*v[2],
        u[0]*v[1] - u[1]*v[0]
    });
}

template <typename T>
vector<T> Vector<T>::hadamardProduct(vector<T> const u, vector<T> const v){
    vector<T> w;
    size_t n = u.size();
    for(size_t i = 0 ; i < n ; i++){
        w.push_back(u[i]*v[i]);
    }
    return w;
}

template <typename T>
T Vector<T>::squareNorm(vector<T> const u){
    T norm = 0;
    size_t n = u.size();
    for(size_t i = 0 ; i < n ; i++){
        norm += u[i]*u[i];
    }
    return norm;
}

template <typename T>
T Vector<T>::norm(vector<T> const u){
    return sqrt(squareNorm(u));
}

template <typename T>
vector<T> Vector<T>::normalize(vector<T> const u){
    return normalize(u, norm(u));
}

template <typename T>
vector<T> Vector<T>::normalize(vector<T> const u, T const norm){
    return scalarDivide(u, norm);
}

template <typename T>
void Vector<T>::normalizeInPlace(vector<T> &u){
    return normalizeInPlace(u, norm(u));
}

template <typename T>
void Vector<T>::normalizeInPlace(vector<T> &u, T const norm){
    scalarDivideInPlace(u, norm);
}

template <typename T>
T Vector<T>::angle(
    vector<T> const u,
    vector<T> const v,
    bool alreadyNormalized
){
    if(alreadyNormalized) return acos(dotProduct(u, v));
    return acos(dotProduct(u,v) / norm(u) / norm(v));
}

template <typename T>
T Vector<T>::acuteAngle(
    vector<T> const u,
    vector<T> const v,
    bool alreadyNormalized
){
    T theta;
    T dot;
    if(alreadyNormalized) dot = dotProduct(u, v);
    else dot = dotProduct(u,v) / norm(u) / norm(v);
    if(dot > 1.0) dot = 1.0;
    if(dot < -1.0) dot = -1.0;
    theta = acos(dot);
    if(theta > M_PI_2) return M_PI - theta;
    return theta;
}

template <typename T>
T Vector<T>::toAngle2D(vector<T> const u){
    return atan2(u[1], u[0]);
}

template <typename T>
vector<T> Vector<T>::findOrthogonal(vector<T> const v){
    // Vector dimensionality
    size_t n = v.size();

    // Find non-zero index for v
    size_t i = 0;
    if(v[0] == 0.0){
        for(size_t j = 1 ; j < n ; ++j){
            if(v[j] != 0.0){
                i = j;
                break;
            }
        }
        if(i==0){
            // Exception, v is a null vector
            throw SurfaceInspectorException(
                "Exception at Vector::findOrthogonal(vector<T> const) : "
                "cannot handle null vectors"
            );
        }
    }

    // Solve orthogonal vector
    vector<T> u(n);
    for(size_t j = 0 ; j < n ; ++j) u[j] = 1.0;
    u[i] = 0.0;
    for(size_t j = 0 ; j < n ; ++j){
        if (j==i) continue;
        u[i] += -v[j];
    }
    u[i] /= v[i];

    // Return
    return u;
}

template <typename T>
vector<vector<T>> Vector<T>::xyRotations(
    size_t const depth,
    bool complement
){
    // Prepare
    T step = M_PI/((T)depth);
    vector<vector<T>> rots(depth);

    // Compute semi-circumference space rotations
    for(size_t i = 0 ; i < depth ; ++i){
        T theta = i*step;
        rots[i] = vector<T>(2);
        rots[i][0] = std::cos(theta);
        rots[i][1] = std::sin(theta);
    }

    // Compute circumference space rotations through negation
    if(complement){
        for(size_t i = 0 ; i < depth ; ++i){
           rots.push_back(negate(rots[i]));
        }
    }

    // Return
    return rots;
}

template <typename T>
vector<vector<T>> Vector<T>::findOrthonormals(vector<vector<T>> const V){
    vector<vector<T>> orthonormals;
    for(vector<T> const & v : V){
        orthonormals.push_back(findOrthonormal(v));
    }
    return orthonormals;
}
template <typename T>
bool Vector<T>::isNull(vector<T> const &v){
    size_t n = v.size();
    for(size_t i = 0 ; i < n ; ++i){
        if(v[i] != 0.0 && v[i] != -0.0) return false;
    }
    return true;
}

template <typename T>
vector<T> Vector<T>::project(vector<T> const v, vector<T> const u){
    return scalarMultiply(u, dotProduct(v,u) / dotProduct(u, u));
}

template <typename T>
vector<T> Vector<T>::project(vector<T> const v, vector<vector<T>> const u){
    vector<T> proj = project(v, u[0]);
    size_t n = u.size();
    for(size_t i = 1 ; i < n ; ++i) proj = add(proj, project(v, u[i]));
    return proj;
}

template <typename T>
vector<T> Vector<T>::diff(vector<T> const u){
    size_t const n = u.size();
    size_t const n_1 = n-1;
    vector<T> v(n_1);
    for(size_t i = 0 ; i < n_1 ; ++i) v[i] = u[i+1] - u[i];
    return v;
}
