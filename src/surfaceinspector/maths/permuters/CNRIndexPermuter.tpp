#ifndef _SURFACEINSPECTOR_MATHS_PERMUTERS_CNRINDEXPERMUTER_HPP_
#include <surfaceinspector/maths/permuters/CNRIndexPermuter.hpp>
#endif

#include <surfaceinspector/maths/Scalar.hpp>

using SurfaceInspector::maths::permuters::CNRIndexPermuter;
using SurfaceInspector::maths::Scalar;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
template <typename T>
CNRIndexPermuter<T>::CNRIndexPermuter(T n, T k) :
    n(n),
    k(k)
{
    max = Scalar<T>::combinationsNoRepetition(n, k);
    stopIndex = n-k;
    current = max;
}

// ***  IPERMUTER  *** //
// ******************* //
template <typename T>
vector<T> CNRIndexPermuter<T>::next(){
    // Handle no remaining permutations scenario
    if(!hasNext()) return indices;

    // Handle initial permutation
    if(indices.empty()){
        indices.resize(k);
        for(T i = 0 ; i < k ; ++i) indices[i] = i;
        ++current;
        return indices;
    }

    // Handle non initial permutations
    T idx = k-1;
    ++indices[idx];
    while((indices[idx] >= n) && (indices[0] <= stopIndex)) {
        --idx;
        ++indices[idx];
        if ((indices[idx] < n) && (indices[idx] < indices[idx + 1])) {
            for (T i = idx + 1; i < k; ++i) {
                indices[i] = indices[i - 1] + 1;
            }
            idx = k - 1;
        }
    }
    ++current;
    return indices;
}
