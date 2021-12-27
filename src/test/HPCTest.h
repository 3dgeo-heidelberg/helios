#pragma once

#include <BaseTest.h>
#include <hpc/SM_ParallelMergeSort.h>

#include <algorithm>

using namespace helios::hpc;

namespace HeliosTests{
/**
 * @author Alberto M. Esmoris Pena
 * @verison 1.0
 * @brief High performance computing components test
 */
class HPCTest : public BaseTest{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //

    // ***  CONSTRUCTOR  *** //
    // ********************* //
    /**
     * @brief HPC test constructor
     */
    HPCTest() : BaseTest("HPC test"){}

    // ***  R U N  *** //
    // *************** //
    bool run() override;

    // ***  SHARED-MEMORY TESTS  *** //
    // ***************************** //
    /**
     * @brief Test shared memory based parallel merge sort implementation
     * @return True if passed, false otherwise
     */
    bool SM_testParallelMergeSort();
};

// ***  COMPARATORS  *** //
// ********************* //
struct HPCTest_IntComparator{
    bool operator() (int i, int j) {return (i<j);}
};
struct HPCTest_FloatComparator{
    bool operator() (float i, float j) {return (i<j);}
};
struct HPCTest_DoubleComparator{
    bool operator() (double i, double j) {return (i<j);}
};

// ***  R U N  *** //
// *************** //
bool HPCTest::run(){
    // Run shared-memory tests
    if(!SM_testParallelMergeSort()) return false;

    // All test passed
    return true;
}

// ***  SHARED-MEMORY TESTS  *** //
// ***************************** //
bool HPCTest::SM_testParallelMergeSort(){
    // Build data to be sorted
    std::vector<int> X1({1, 7, 6, 5, 2, 3, 9, 11, 25, -7, -8, 3, -3, 6});
    std::vector<float> X2({1.0, -1.0, 2.0});
    std::vector<double> X3({-1.0, 1.0, -2.0, 2.0, 3.5, 2.5, 7.1, -3.6, 2.88});

    // Build expected output
    std::vector<int> Y1({-8, -7, -3, 1, 2, 3, 3, 5, 6, 6, 7, 9, 11, 25});
    std::vector<float> Y2({-1.0, 1.0, 2.0});
    std::vector<double> Y3({-3.6, -2.0, -1.0, 1.0, 2.0, 2.5, 2.88, 3.5, 7.1});

    // Build sorters
    SM_ParallelMergeSort<std::vector<int>::iterator, HPCTest_IntComparator>
        PMS2T1(2, 4);  // 2 Threads, for ints
    SM_ParallelMergeSort<std::vector<float>::iterator, HPCTest_FloatComparator>
        PMS2T2(2, 4);  // 2 Threads, for floats
    SM_ParallelMergeSort<
        std::vector<double>::iterator,
        HPCTest_DoubleComparator
    > PMS2T3(2, 4);  // 2 Threads, for doubles
    SM_ParallelMergeSort<std::vector<int>::iterator, HPCTest_IntComparator>
        PMS3T1(3, 6);  // 3 Threads, for ints
    SM_ParallelMergeSort<std::vector<float>::iterator, HPCTest_FloatComparator>
        PMS3T2(3, 6);  // 3 Threads, for floats
    SM_ParallelMergeSort<
        std::vector<double>::iterator,
        HPCTest_DoubleComparator
    > PMS3T3(3, 6);  // 3 Threads, for doubles

    // Test 2 threads sorter
    std::vector<int> Z1(X1);
    PMS2T1.sort(Z1.begin(), Z1.end(), HPCTest_IntComparator());
    if(Y1.size() != Z1.size()) return false;
    if(!std::equal(Y1.begin(), Y1.end(), Z1.begin())) return false;
    std::vector<float> Z2(X2);
    PMS2T2.sort(Z2.begin(), Z2.end(), HPCTest_FloatComparator());
    if(Y2.size() != Z2.size()) return false;
    if(!std::equal(Y2.begin(), Y2.end(), Z2.begin())) return false;
    std::vector<double> Z3(X3);
    PMS2T3.sort(Z3.begin(), Z3.end(), HPCTest_DoubleComparator());
    if(Y3.size() != Z3.size()) return false;
    if(!std::equal(Y3.begin(), Y3.end(), Z3.begin())) return false;

    // Test 3 threads sorter
    Z1 = std::vector<int>(X1);
    PMS3T1.sort(Z1.begin(), Z1.end(), HPCTest_IntComparator());
    if(Y1.size() != Z1.size()) return false;
    if(!std::equal(Y1.begin(), Y1.end(), Z1.begin())) return false;
    Z2 = std::vector<float>(X2);
    PMS3T2.sort(Z2.begin(), Z2.end(), HPCTest_FloatComparator());
    if(Y2.size() != Z2.size()) return false;
    if(!std::equal(Y2.begin(), Y2.end(), Z2.begin())) return false;
    Z3 = std::vector<double>(X3);
    PMS3T3.sort(Z3.begin(), Z3.end(), HPCTest_DoubleComparator());
    if(Y3.size() != Z3.size()) return false;
    if(!std::equal(Y3.begin(), Y3.end(), Z3.begin())) return false;

    // All checks passed
    return true;
}

}