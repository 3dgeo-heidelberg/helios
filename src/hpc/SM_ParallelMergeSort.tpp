#ifndef _SM_PARALLEL_MERGE_SORT_H_
#include <SM_ParallelMergeSort.h>
#endif

#include <SM_ParallelMergeSortSubTask.h>

#include <algorithm>
#include <iterator>


// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
template <typename RandomAccessIterator, typename Comparator>
void helios::hpc::SM_ParallelMergeSort<RandomAccessIterator, Comparator>::init(){
    maxDepth = (int) std::floor(std::log2(numThreads)); // d^*
    stSequencer = std::make_shared<SharedTaskSequencer>(numThreads);
}

// ***  SORT METHODS  *** //
// ********************** //
template <typename RandomAccessIterator, typename Comparator>
void helios::hpc::SM_ParallelMergeSort<RandomAccessIterator, Comparator>::trySort(
    RandomAccessIterator begin,
    RandomAccessIterator end,
    Comparator comparator
){
    stSequencer->start(std::make_shared<
        helios::hpc::SM_ParallelMergeSortSubTask<RandomAccessIterator, Comparator>
    >(
        stSequencer,
        0,
        numThreads,
        minElements,
        maxDepth,
        begin,
        end,
        comparator
    ));
}

// ***  CONTROL METHODS  *** //
// ************************* //
template <typename RandomAccessIterator, typename Comparator>
void helios::hpc::SM_ParallelMergeSort<RandomAccessIterator, Comparator>::join(){
    stSequencer->joinAll();
}
