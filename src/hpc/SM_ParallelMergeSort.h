#ifndef _SM_PARALLEL_MERGE_SORT_H_
#define _SM_PARALLEL_MERGE_SORT_H_

#include <SharedTaskSequencer.h>

#include <memory>

namespace helios{ namespace hpc {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class providing a shared memory sorting algorithm based on merge
 *  sort.
 *
 * Sort \f$m\f$ elements using \f$n\f$ threads.
 *
 * @tparam RandomAccessIterator Type of iterator specifying start and end for
 *  sequence to be sorted
 * @tparam Comparator Type of comparator to be used to compare elements from
 *  a sequence
 */
template <typename RandomAccessIterator, typename Comparator>
class SM_ParallelMergeSort {
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief How many threads can be used to compute parallel sorting
     *
     * \f[
     *  n
     * \f]
     */
    size_t numThreads;
    /**
     * @brief If the number of elements to be sorted is less than minElements,
     *  then a sequential sort will be applied
     *
     * \f[
     *  m_*
     * \f]
     */
    size_t minElements;
    /**
     * @brief The maximum depth that the tree of splits must reach considering
     *  number of threads
     *
     * \f[
     *  d^*
     * \f]
     *
     * @see SM_ParallelMergeSort::numThreads
     */
    int maxDepth;
    /**
     * @brief The shared task sequencer to handle concurrent execution of
     *  multiple threads
     * @see SharedTaskSequencer
     */
    std::shared_ptr<SharedTaskSequencer> stSequencer;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Build a shared memory parallel merge sort instance with given
     *  number of threads. The number of minimum elements will be twice the
     *  number of threads.
     * @see SM_ParallelMergeSort::numThreads
     */
    SM_ParallelMergeSort(size_t const numThreads) :
        numThreads(numThreads),
        minElements(2*numThreads)
    {init();}
    /**
     * @brief Build a shared memory parallel merge sort instance with given
     *  number of threads and given minimum number of elements.
     * @see SM_ParallelMergeSort::numThreads
     * @see SM_ParallelMergeSort::minElements
     */
    SM_ParallelMergeSort(size_t const numThreads, size_t const minElements) :
        numThreads(numThreads),
        minElements(minElements)
    {init();}
    /**
     * @brief Common initialization for all constructors
     */
    virtual void init();
    virtual ~SM_ParallelMergeSort() {}

    // ***  SORT METHODS  *** //
    // ********************** //
    /**
     * @brief Sort given sequence using given comparator in a non-blocking
     *  fashion. It is, the function might return before the sequence has
     *  been sorted.
     * @param begin Start of sequence to be sorted
     * @param end End of sequence to be sorted
     * @param comparator Comparator defining sorting criteria
     * @see SM_ParallelMergeSort::sort
     */
    virtual void trySort(
        RandomAccessIterator begin,
        RandomAccessIterator end,
        Comparator comparator
    );
    /**
     * @brief Like SM_ParallelMergeSort::trySort function but it returns only
     *  after sorting has been finished. It can be considered as the blocking
     *  counterpart of trySort
     * @see SM_ParallelMergeSort::trySort
     */
    virtual inline void sort(
        RandomAccessIterator begin,
        RandomAccessIterator end,
        Comparator comparator
    ){
        trySort(begin, end, comparator);
        join();
    }

    // ***  CONTROL METHODS  *** //
    // ************************* //
    /**
     * @brief Force caller thread to wait until current sort has been finished
     *
     * <b>Usage example</b>
     * @code
     *  SM_ParallelMergeSort<...> sorter(4, 32);
     *  sorter.trySort(x.begin(), x.end(), MyComparator())
     *  ...
     *  sorter.join()
     * @endcode
     */
    virtual void join();

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Get the number of threads
     * @return Number of threads
     * @see SM_ParallelMergeSort::numThreads
     */
    virtual inline size_t getNumThreads() const {return numThreads;}
    /**
     * @brief Set the number of threads
     * @param numThreads New number of threads
     * @see SM_ParallelMergeSort::numThreads
     */
    virtual inline void setNumThreads(size_t const numThreads)
    {this->numThreads = numThreads;}
    /**
     * @brief Get the minimum number of elements for parallel execution
     * @return Minimum number of elements for parallel execution
     * @see SM_ParallelMergeSort::minElements
     */
    virtual inline size_t getMinElements() const {return minElements;}
    /**
     * @brief Set the minimum number of elements for parallel execution
     * @param minElements New minimum number of elements for parallel execution
     * @see SM_ParallelMergeSort::minElements
     */
    virtual inline void setMinElements(size_t const minElements)
    {this->minElements = minElements;}
};


}}

#include <hpc/SM_ParallelMergeSort.tpp>

#endif

