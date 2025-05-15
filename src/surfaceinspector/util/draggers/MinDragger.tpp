#ifndef _SURFACEINSPECTOR_UTIL_DRAGGERS_MIN_DRAGGER_HPP_
#include <surfaceinspector/util/draggers/MinDragger.hpp>
#endif

using SurfaceInspector::util::draggers::MinDragger;

// ***  INITIALIZATION  *** //
// ************************ //
template <typename E> void MinDragger<E>::initialize(){
    a = 0;
    b = x.size()-1;
    c = 0;
    partialSort();
    initialized = true;
}

// ***  INNER METHODS  *** //
// *********************** //
template <typename E> void MinDragger<E>::partialSort(){
    // Prepare variables
    E alpha = x[a];  // alpha = xmin
    E beta = alpha;  // beta = xmax
    E xi;
    size_t alphaIdx = a, betaIdx = a;

    // Find alpha and beta
    for(size_t i = a+1 ; i <= b ; ++i){
        xi = x[i];
        if(xi < alpha){
            alpha = xi;
            alphaIdx = i;
        }
        if(xi > beta) {
            beta = xi;
            betaIdx = i;
        }
    }

    // Handle special case : a index equals beta index
    if(a==betaIdx) betaIdx = alphaIdx; // After alpha swap, betaIdx -> alphaIdx

    // Swap alpha
    xi = x[a];
    x[a] = alpha;
    x[alphaIdx] = xi;

    // Swap beta
    xi = x[b];
    x[b] = beta;
    x[betaIdx] = xi;
}

// ***  OPTIMIZATION DRAGGER METHODS  *** //
// ************************************** //
template <typename E> void MinDragger<E>::update(){
    if(!initialized) initialize(); // First time, initialize
    else{ // After first time
        if(a<b){ // Update (a,b) indices and partial sort if necessary
            ++a;
            --b;
            if(a>b) b=a; // Prevent indices from moving after intersection
            partialSort();
        }
        ++c; // Update index of current element
    }
}
