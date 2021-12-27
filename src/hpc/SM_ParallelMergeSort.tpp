#ifndef _SM_PARALLEL_MERGE_SORT_H_

#include <algorithm>
#include <iterator>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
template <typename RandomAccessIterator, typename Comparator>
void SM_ParallelMergeSort<RandomAccessIterator, Comparator>::init(){
    maxDepth = (int) std::floor(std::log2(numThreads)); // d^*
}

// ***  SORT METHODS  *** //
// ********************** //
template <typename RandomAccessIterator, typename Comparator>
void SM_ParallelMergeSort<RandomAccessIterator, Comparator>::trySort(
    RandomAccessIterator begin,
    RandomAccessIterator end,
    Comparator comparator
){
    // Handle sequential sort cases
    size_t const numElements = std::distance(begin, end); // m
    if(numElements < minElements || numThreads==1){
        std::sort(begin, end, comparator);
    }

    // TODO Rethink : Implement handling of parallel cases
    std::sort(begin, end, comparator); // TODO Remove
}

// ***  CONTROL METHODS  *** //
// ************************* //
template <typename RandomAccessIterator, typename Comparator>
void SM_ParallelMergeSort<RandomAccessIterator, Comparator>::join(){
    // TODO Rethink : Implement join mechanism
}

#endif
