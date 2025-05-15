#include <catch2/catch_test_macros.hpp>
#include <hpc/SM_ParallelMergeSort.h>

#include <algorithm>

using namespace helios::hpc;

TEST_CASE( "HPC test" ) {
    struct HPCTest_IntComparator{
        bool operator() (int i, int j) {return (i<j);}
    };
    struct HPCTest_FloatComparator{
        bool operator() (float i, float j) {return (i<j);}
    };
    struct HPCTest_DoubleComparator{
        bool operator() (double i, double j) {return (i<j);}
    };

    // Build data to be sorted
    std::vector<int> X1({1, 7, 6, 5, 2, 3, 9, 11, 25, -7, -8, 3, -3, 6});
    std::vector<float> X2({1.0, -1.0, 2.0});
    std::vector<double> X3({-1.0, 1.0, -2.0, 2.0, 3.5, 2.5, 7.1, -3.6, 2.88});
    std::vector<int> X4({
        22, 12, 27, 14, 31, 21, 12, 21,  3,  0, 14,  5,  3, 28,  8,  9, 23,
        30, 22,  8, 23, 16,  7, 16, 14,  7,  6,  2,  8, 15,  2,  3,  0,  6,
        6, 11, 10,  0,  7,  7, 11, 28, 28, 21, 23, 16, 18, 17,  8, 23,  1,
        29,  2, 17, 16, 17, 11, 13, 13,  3,  4, 20, 20, 15, 18,  0,  0, 27,
        14, 11, 18, 10, 14,  6,  6, 25, 11, 18,  3,  5, 10,  2, 30, 24,  9,
        13, 24, 18,  9, 31,  1, 19,  3,  2, 12, 30, 15, 21, 15, 23, 20, 26,
        25, 17, 16, 13, 25, 18,  1, 15,  7,  8, 22, 15,  0,  8, 19, 17,  8,
        15, 16, 19, 16,  4,  5,  8, 21, 16,  1, 16, 21, 21, 27, 18, 10,  5,
        14,  6,  5, 27, 31, 28,  5,  5, 11,  1,  8, 17,  2, 18,  8, 22, 28,
        3, 12,  7,  9, 17, 13,  0, 10, 26, 28, 13, 14, 21, 16,  7, 11, 14,
        28, 11, 28, 22,  8,  9, 29,  8, 28, 10, 12, 31, 27, 24, 20,  2, 23,
        8, 13,  0, 23,  0, 19, 11,  4,  9,  8, 24,  8, 28
    });
    std::vector<int> X5({
        22, 12, 27, 14, 31, 21, 12, 21,  3,  0, 14,  5,  3, 28,  8,  9, 23,
        30, 22,  8, 23, 16,  7, 16, 14,  7,  6,  2,  8, 15,  2,  3,  0,  6,
        6, 11, 10,  0,  7,  7, 11, 28, 28, 21, 23, 16, 18, 17,  8, 23,  1,
        29,  2, 17, 16, 17, 11, 13, 13,  3,  4, 20, 20, 15, 18,  0,  0, 27,
        14, 11, 18, 10, 14,  6,  6, 25, 11, 18,  3,  5, 10,  2, 30, 24,  9,
        13, 24, 18,  9, 31,  1, 19,  3,  2, 12, 30, 15, 21, 15, 23, 20, 26,
        25, 17, 16, 13, 25, 18,  1, 15,  7,  8, 22, 15,  0,  8, 19, 17,  8,
        15, 16, 19, 16,  4,  5,  8, 21, 16,  1, 16, 21, 21, 27, 18, 10,  5,
        14,  6,  5, 27, 31, 28,  5,  5, 11,  1,  8, 17,  2, 18,  8, 22, 28,
        3, 12,  7,  9, 17, 13,  0, 10, 26, 28, 13, 14, 21, 16,  7, 11, 14,
        28, 11, 28, 22,  8,  9, 29,  8, 28, 10, 12, 31, 27, 24, 20,  2, 23,
        8, 13,  0, 23,  0, 19, 11,  4,  9,  8, 24,  8
    });

    // Build expected output
    std::vector<int> Y1({-8, -7, -3, 1, 2, 3, 3, 5, 6, 6, 7, 9, 11, 25});
    std::vector<float> Y2({-1.0, 1.0, 2.0});
    std::vector<double> Y3({-3.6, -2.0, -1.0, 1.0, 2.0, 2.5, 2.88, 3.5, 7.1});
    std::vector<int> Y4({
        0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  2,  2,  2,
        2,  2,  2,  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  5,  5,  5,
        5,  5,  5,  5,  6,  6,  6,  6,  6,  6,  7,  7,  7,  7,  7,  7,  7,
        8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  9,  9,
        9,  9,  9,  9, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11,
        11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14,
        14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16,
        16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18,
        18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 21, 21, 21,
        21, 21, 22, 22, 22, 22, 22, 23, 23, 23, 23, 23, 23, 23, 24, 24, 24,
        24, 25, 25, 25, 26, 26, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 28,
        28, 28, 28, 28, 29, 29, 30, 30, 30, 31, 31, 31, 31
    });
    std::vector<int> Y5({
        0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  2,  2,  2,
        2,  2,  2,  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  5,  5,  5,
        5,  5,  5,  5,  6,  6,  6,  6,  6,  6,  7,  7,  7,  7,  7,  7,  7,
        8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  9,  9,
        9,  9,  9,  9, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11,
        11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14,
        14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16,
        16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18,
        18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 21, 21, 21,
        21, 21, 22, 22, 22, 22, 22, 23, 23, 23, 23, 23, 23, 23, 24, 24, 24,
        24, 25, 25, 25, 26, 26, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28,
        28, 28, 28, 28, 29, 29, 30, 30, 30, 31, 31, 31, 31
    });

    std::vector<int> Z1(X1);
    std::vector<float> Z2(X2);
    std::vector<double> Z3(X3);
    std::vector<int> Z4(X4);
    std::vector<int> Z5(X5);

    Z1 = std::vector<int>(X1);
    Z2 = std::vector<float>(X2);
    Z3 = std::vector<double>(X3);
    Z4 = std::vector<int>(X4);
    Z5 = std::vector<int>(X5);

    SECTION("2 threads sorter") {
        SM_ParallelMergeSort<std::vector<int>::iterator, HPCTest_IntComparator>
            PMS2T1(2, 4);  // 2 Threads, for ints
        SM_ParallelMergeSort<std::vector<float>::iterator, HPCTest_FloatComparator>
            PMS2T2(2, 4);  // 2 Threads, for floats
        SM_ParallelMergeSort<
            std::vector<double>::iterator,
            HPCTest_DoubleComparator
        > PMS2T3(2, 4);  // 2 Threads, for doubles

        // Test 2 threads sorter
        PMS2T1.sort(Z1.begin(), Z1.end(), HPCTest_IntComparator());
        REQUIRE(Y1.size() == Z1.size());
        REQUIRE(std::equal(Y1.begin(), Y1.end(), Z1.begin()));
        PMS2T2.sort(Z2.begin(), Z2.end(), HPCTest_FloatComparator());
        REQUIRE(Y2.size() == Z2.size());
        REQUIRE(std::equal(Y2.begin(), Y2.end(), Z2.begin()));
        PMS2T3.sort(Z3.begin(), Z3.end(), HPCTest_DoubleComparator());
        REQUIRE(Y3.size() == Z3.size());
        REQUIRE(std::equal(Y3.begin(), Y3.end(), Z3.begin()));
        PMS2T1.sort(Z4.begin(), Z4.end(), HPCTest_IntComparator());
        REQUIRE(Y4.size() == Z4.size());
        REQUIRE(std::equal(Y4.begin(), Y4.end(), Z4.begin()));
        PMS2T1.sort(Z5.begin(), Z5.end(), HPCTest_IntComparator());
        REQUIRE(Y5.size() == Z5.size());
        REQUIRE(std::equal(Y5.begin(), Y5.end(), Z5.begin()));
    }

    SECTION("3 threads sorter") {
        SM_ParallelMergeSort<std::vector<int>::iterator, HPCTest_IntComparator>
            PMS3T1(3, 6);  // 3 Threads, for ints
        SM_ParallelMergeSort<std::vector<float>::iterator, HPCTest_FloatComparator>
            PMS3T2(3, 6);  // 3 Threads, for floats
        SM_ParallelMergeSort<
            std::vector<double>::iterator,
            HPCTest_DoubleComparator
        > PMS3T3(3, 6);  // 3 Threads, for doubles

        // Test 3 threads sorter
        PMS3T1.sort(Z1.begin(), Z1.end(), HPCTest_IntComparator());
        REQUIRE(Y1.size() == Z1.size());
        REQUIRE(std::equal(Y1.begin(), Y1.end(), Z1.begin()));
        PMS3T2.sort(Z2.begin(), Z2.end(), HPCTest_FloatComparator());
        REQUIRE(Y2.size() == Z2.size());
        REQUIRE(std::equal(Y2.begin(), Y2.end(), Z2.begin()));
        PMS3T3.sort(Z3.begin(), Z3.end(), HPCTest_DoubleComparator());
        REQUIRE(Y3.size() == Z3.size());
        REQUIRE(std::equal(Y3.begin(), Y3.end(), Z3.begin()));
        PMS3T1.sort(Z4.begin(), Z4.end(), HPCTest_IntComparator());
        REQUIRE(Y4.size() == Z4.size());
        REQUIRE(std::equal(Y4.begin(), Y4.end(), Z4.begin()));
        PMS3T1.sort(Z5.begin(), Z5.end(), HPCTest_IntComparator());
        REQUIRE(Y5.size() == Z5.size());
        REQUIRE(std::equal(Y5.begin(), Y5.end(), Z5.begin()));
    }

    SECTION("11 threads sorter") {
        SM_ParallelMergeSort<std::vector<int>::iterator, HPCTest_IntComparator>
            PMS11T1(11, 22);  // 11 Threads, for ints
        SM_ParallelMergeSort<std::vector<float>::iterator, HPCTest_FloatComparator>
            PMS11T2(11, 22);  // 11 Threads, for floats
        SM_ParallelMergeSort<
            std::vector<double>::iterator,
            HPCTest_DoubleComparator
        > PMS11T3(11, 22);  // 11 Threads, for doubles

        // Test 11 threads sorter
        PMS11T1.sort(Z1.begin(), Z1.end(), HPCTest_IntComparator());
        REQUIRE(Y1.size() == Z1.size());
        REQUIRE(std::equal(Y1.begin(), Y1.end(), Z1.begin()));
        PMS11T2.sort(Z2.begin(), Z2.end(), HPCTest_FloatComparator());
        REQUIRE(Y2.size() == Z2.size());
        REQUIRE(std::equal(Y2.begin(), Y2.end(), Z2.begin()));
        PMS11T3.sort(Z3.begin(), Z3.end(), HPCTest_DoubleComparator());
        REQUIRE(Y3.size() == Z3.size());
        REQUIRE(std::equal(Y3.begin(), Y3.end(), Z3.begin()));
        PMS11T1.sort(Z4.begin(), Z4.end(), HPCTest_IntComparator());
        REQUIRE(Y4.size() == Z4.size());
        REQUIRE(std::equal(Y4.begin(), Y4.end(), Z4.begin()));
        PMS11T1.sort(Z5.begin(), Z5.end(), HPCTest_IntComparator());
        REQUIRE(Y5.size() == Z5.size());
        REQUIRE(std::equal(Y5.begin(), Y5.end(), Z5.begin()));
    }

    SECTION("16 threads sorter") {
        SM_ParallelMergeSort<std::vector<int>::iterator, HPCTest_IntComparator>
            PMS16T1(16, 32);  // 16 Threads, for ints
        SM_ParallelMergeSort<std::vector<float>::iterator, HPCTest_FloatComparator>
            PMS16T2(16, 32);  // 16 Threads, for floats
        SM_ParallelMergeSort<
            std::vector<double>::iterator,
            HPCTest_DoubleComparator
        > PMS16T3(16, 32);  // 16 Threads, for doubles

        // Test 16 threads sorter
        PMS16T1.sort(Z1.begin(), Z1.end(), HPCTest_IntComparator());
        REQUIRE(Y1.size() == Z1.size());
        REQUIRE(std::equal(Y1.begin(), Y1.end(), Z1.begin()));
        PMS16T2.sort(Z2.begin(), Z2.end(), HPCTest_FloatComparator());
        REQUIRE(Y2.size() == Z2.size());
        REQUIRE(std::equal(Y2.begin(), Y2.end(), Z2.begin()));
        PMS16T3.sort(Z3.begin(), Z3.end(), HPCTest_DoubleComparator());
        REQUIRE(Y3.size() == Z3.size());
        REQUIRE(std::equal(Y3.begin(), Y3.end(), Z3.begin()));
        PMS16T1.sort(Z4.begin(), Z4.end(), HPCTest_IntComparator());
        REQUIRE(Y4.size() == Z4.size());
        REQUIRE(std::equal(Y4.begin(), Y4.end(), Z4.begin()));
        PMS16T1.sort(Z5.begin(), Z5.end(), HPCTest_IntComparator());
        REQUIRE(Y5.size() == Z5.size());
        REQUIRE(std::equal(Y5.begin(), Y5.end(), Z5.begin()));
    }
}