// ***  GENERAL PURPOSE  *** //
// ************************* //
template <typename T>
T SurfaceInspector::maths::Scalar<T>::factorial(T const n){
    T fact = 1;
    for(T i = 2 ; i <= n ; ++i) fact *= i;
    return fact;
}

template <typename T>
T SurfaceInspector::maths::Scalar<T>::binom(T const n, T const k){
    return factorial(n) / factorial(k) / factorial(n-k);
}

// ***  COMBINATORY  *** //
// ********************* //
template <typename T>
T SurfaceInspector::maths::Scalar<T>::variationsNoRepetition(T const n, T const k){
    return factorial(n)/factorial(n-k);
}
template <typename T>
T SurfaceInspector::maths::Scalar<T>::variationsRepetition(T const n, T const k){
    T v = 1;
    for(T i = 0 ; i < k ; ++i) v*= n;
    return v;
}
