#pragma once

#include <BaseTest.h>
#include <hpc/SM_ParallelMergeSort.h>

#include <algorithm>

using namespace helios::hpc;

namespace HeliosTests {
/**
 * @author Alberto M. Esmoris Pena
 * @verison 1.0
 * @brief High performance computing components test
 */
class HPCTest : public BaseTest
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //

  // ***  CONSTRUCTOR  *** //
  // ********************* //
  /**
   * @brief HPC test constructor
   */
  HPCTest()
    : BaseTest("HPC test")
  {
  }

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
struct HPCTest_IntComparator
{
  bool operator()(int i, int j) { return (i < j); }
};
struct HPCTest_FloatComparator
{
  bool operator()(float i, float j) { return (i < j); }
};
struct HPCTest_DoubleComparator
{
  bool operator()(double i, double j) { return (i < j); }
};

// ***  R U N  *** //
// *************** //
bool
HPCTest::run()
{
  // Run shared-memory tests
  if (!SM_testParallelMergeSort())
    return false;

  // All test passed
  return true;
}

// ***  SHARED-MEMORY TESTS  *** //
// ***************************** //
bool
HPCTest::SM_testParallelMergeSort()
{
  // Build data to be sorted
  std::vector<int> X1({ 1, 7, 6, 5, 2, 3, 9, 11, 25, -7, -8, 3, -3, 6 });
  std::vector<float> X2({ 1.0, -1.0, 2.0 });
  std::vector<double> X3({ -1.0, 1.0, -2.0, 2.0, 3.5, 2.5, 7.1, -3.6, 2.88 });
  std::vector<int> X4(
    { 22, 12, 27, 14, 31, 21, 12, 21, 3,  0,  14, 5,  3,  28, 8,  9,  23,
      30, 22, 8,  23, 16, 7,  16, 14, 7,  6,  2,  8,  15, 2,  3,  0,  6,
      6,  11, 10, 0,  7,  7,  11, 28, 28, 21, 23, 16, 18, 17, 8,  23, 1,
      29, 2,  17, 16, 17, 11, 13, 13, 3,  4,  20, 20, 15, 18, 0,  0,  27,
      14, 11, 18, 10, 14, 6,  6,  25, 11, 18, 3,  5,  10, 2,  30, 24, 9,
      13, 24, 18, 9,  31, 1,  19, 3,  2,  12, 30, 15, 21, 15, 23, 20, 26,
      25, 17, 16, 13, 25, 18, 1,  15, 7,  8,  22, 15, 0,  8,  19, 17, 8,
      15, 16, 19, 16, 4,  5,  8,  21, 16, 1,  16, 21, 21, 27, 18, 10, 5,
      14, 6,  5,  27, 31, 28, 5,  5,  11, 1,  8,  17, 2,  18, 8,  22, 28,
      3,  12, 7,  9,  17, 13, 0,  10, 26, 28, 13, 14, 21, 16, 7,  11, 14,
      28, 11, 28, 22, 8,  9,  29, 8,  28, 10, 12, 31, 27, 24, 20, 2,  23,
      8,  13, 0,  23, 0,  19, 11, 4,  9,  8,  24, 8,  28 });
  std::vector<int> X5(
    { 22, 12, 27, 14, 31, 21, 12, 21, 3,  0,  14, 5,  3,  28, 8,  9,  23,
      30, 22, 8,  23, 16, 7,  16, 14, 7,  6,  2,  8,  15, 2,  3,  0,  6,
      6,  11, 10, 0,  7,  7,  11, 28, 28, 21, 23, 16, 18, 17, 8,  23, 1,
      29, 2,  17, 16, 17, 11, 13, 13, 3,  4,  20, 20, 15, 18, 0,  0,  27,
      14, 11, 18, 10, 14, 6,  6,  25, 11, 18, 3,  5,  10, 2,  30, 24, 9,
      13, 24, 18, 9,  31, 1,  19, 3,  2,  12, 30, 15, 21, 15, 23, 20, 26,
      25, 17, 16, 13, 25, 18, 1,  15, 7,  8,  22, 15, 0,  8,  19, 17, 8,
      15, 16, 19, 16, 4,  5,  8,  21, 16, 1,  16, 21, 21, 27, 18, 10, 5,
      14, 6,  5,  27, 31, 28, 5,  5,  11, 1,  8,  17, 2,  18, 8,  22, 28,
      3,  12, 7,  9,  17, 13, 0,  10, 26, 28, 13, 14, 21, 16, 7,  11, 14,
      28, 11, 28, 22, 8,  9,  29, 8,  28, 10, 12, 31, 27, 24, 20, 2,  23,
      8,  13, 0,  23, 0,  19, 11, 4,  9,  8,  24, 8 });

  // Build expected output
  std::vector<int> Y1({ -8, -7, -3, 1, 2, 3, 3, 5, 6, 6, 7, 9, 11, 25 });
  std::vector<float> Y2({ -1.0, 1.0, 2.0 });
  std::vector<double> Y3({ -3.6, -2.0, -1.0, 1.0, 2.0, 2.5, 2.88, 3.5, 7.1 });
  std::vector<int> Y4(
    { 0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  2,  2,  2,
      2,  2,  2,  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  5,  5,  5,
      5,  5,  5,  5,  6,  6,  6,  6,  6,  6,  7,  7,  7,  7,  7,  7,  7,
      8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  9,  9,
      9,  9,  9,  9,  10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11,
      11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14,
      14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16,
      16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18,
      18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 21, 21, 21,
      21, 21, 22, 22, 22, 22, 22, 23, 23, 23, 23, 23, 23, 23, 24, 24, 24,
      24, 25, 25, 25, 26, 26, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 28,
      28, 28, 28, 28, 29, 29, 30, 30, 30, 31, 31, 31, 31 });
  std::vector<int> Y5(
    { 0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  2,  2,  2,
      2,  2,  2,  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  5,  5,  5,
      5,  5,  5,  5,  6,  6,  6,  6,  6,  6,  7,  7,  7,  7,  7,  7,  7,
      8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  9,  9,
      9,  9,  9,  9,  10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11,
      11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14,
      14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16,
      16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18,
      18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 21, 21, 21,
      21, 21, 22, 22, 22, 22, 22, 23, 23, 23, 23, 23, 23, 23, 24, 24, 24,
      24, 25, 25, 25, 26, 26, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 28,
      28, 28, 28, 29, 29, 30, 30, 30, 31, 31, 31, 31 });

  // Build sorters
  SM_ParallelMergeSort<std::vector<int>::iterator, HPCTest_IntComparator>
    PMS2T1(2, 4); // 2 Threads, for ints
  SM_ParallelMergeSort<std::vector<float>::iterator, HPCTest_FloatComparator>
    PMS2T2(2, 4); // 2 Threads, for floats
  SM_ParallelMergeSort<std::vector<double>::iterator, HPCTest_DoubleComparator>
    PMS2T3(2, 4); // 2 Threads, for doubles
  SM_ParallelMergeSort<std::vector<int>::iterator, HPCTest_IntComparator>
    PMS3T1(3, 6); // 3 Threads, for ints
  SM_ParallelMergeSort<std::vector<float>::iterator, HPCTest_FloatComparator>
    PMS3T2(3, 6); // 3 Threads, for floats
  SM_ParallelMergeSort<std::vector<double>::iterator, HPCTest_DoubleComparator>
    PMS3T3(3, 6); // 3 Threads, for doubles
  SM_ParallelMergeSort<std::vector<int>::iterator, HPCTest_IntComparator>
    PMS11T1(11, 22); // 11 Threads, for ints
  SM_ParallelMergeSort<std::vector<float>::iterator, HPCTest_FloatComparator>
    PMS11T2(11, 22); // 11 Threads, for floats
  SM_ParallelMergeSort<std::vector<double>::iterator, HPCTest_DoubleComparator>
    PMS11T3(11, 22); // 11 Threads, for doubles
  SM_ParallelMergeSort<std::vector<int>::iterator, HPCTest_IntComparator>
    PMS16T1(16, 32); // 16 Threads, for ints
  SM_ParallelMergeSort<std::vector<float>::iterator, HPCTest_FloatComparator>
    PMS16T2(16, 32); // 16 Threads, for floats
  SM_ParallelMergeSort<std::vector<double>::iterator, HPCTest_DoubleComparator>
    PMS16T3(16, 32); // 16 Threads, for doubles

  // Test 2 threads sorter
  std::vector<int> Z1(X1);
  PMS2T1.sort(Z1.begin(), Z1.end(), HPCTest_IntComparator());
  if (Y1.size() != Z1.size())
    return false;
  if (!std::equal(Y1.begin(), Y1.end(), Z1.begin()))
    return false;
  std::vector<float> Z2(X2);
  PMS2T2.sort(Z2.begin(), Z2.end(), HPCTest_FloatComparator());
  if (Y2.size() != Z2.size())
    return false;
  if (!std::equal(Y2.begin(), Y2.end(), Z2.begin()))
    return false;
  std::vector<double> Z3(X3);
  PMS2T3.sort(Z3.begin(), Z3.end(), HPCTest_DoubleComparator());
  if (Y3.size() != Z3.size())
    return false;
  if (!std::equal(Y3.begin(), Y3.end(), Z3.begin()))
    return false;
  std::vector<int> Z4(X4);
  PMS2T1.sort(Z4.begin(), Z4.end(), HPCTest_IntComparator());
  if (Y4.size() != Z4.size())
    return false;
  if (!std::equal(Y4.begin(), Y4.end(), Z4.begin()))
    return false;
  std::vector<int> Z5(X5);
  PMS2T1.sort(Z5.begin(), Z5.end(), HPCTest_IntComparator());
  if (Y5.size() != Z5.size())
    return false;
  if (!std::equal(Y5.begin(), Y5.end(), Z5.begin()))
    return false;

  // Test 3 threads sorter
  Z1 = std::vector<int>(X1);
  PMS3T1.sort(Z1.begin(), Z1.end(), HPCTest_IntComparator());
  if (Y1.size() != Z1.size())
    return false;
  if (!std::equal(Y1.begin(), Y1.end(), Z1.begin()))
    return false;
  Z2 = std::vector<float>(X2);
  PMS3T2.sort(Z2.begin(), Z2.end(), HPCTest_FloatComparator());
  if (Y2.size() != Z2.size())
    return false;
  if (!std::equal(Y2.begin(), Y2.end(), Z2.begin()))
    return false;
  Z3 = std::vector<double>(X3);
  PMS3T3.sort(Z3.begin(), Z3.end(), HPCTest_DoubleComparator());
  if (Y3.size() != Z3.size())
    return false;
  if (!std::equal(Y3.begin(), Y3.end(), Z3.begin()))
    return false;
  Z4 = std::vector<int>(X4);
  PMS3T1.sort(Z4.begin(), Z4.end(), HPCTest_IntComparator());
  if (Y4.size() != Z4.size())
    return false;
  if (!std::equal(Y4.begin(), Y4.end(), Z4.begin()))
    return false;
  Z5 = std::vector<int>(X5);
  PMS3T1.sort(Z5.begin(), Z5.end(), HPCTest_IntComparator());
  if (Y5.size() != Z5.size())
    return false;
  if (!std::equal(Y5.begin(), Y5.end(), Z5.begin()))
    return false;

  // Test 11 threads sorter
  Z1 = std::vector<int>(X1);
  PMS11T1.sort(Z1.begin(), Z1.end(), HPCTest_IntComparator());
  if (Y1.size() != Z1.size())
    return false;
  if (!std::equal(Y1.begin(), Y1.end(), Z1.begin()))
    return false;
  Z2 = std::vector<float>(X2);
  PMS11T2.sort(Z2.begin(), Z2.end(), HPCTest_FloatComparator());
  if (Y2.size() != Z2.size())
    return false;
  if (!std::equal(Y2.begin(), Y2.end(), Z2.begin()))
    return false;
  Z3 = std::vector<double>(X3);
  PMS11T3.sort(Z3.begin(), Z3.end(), HPCTest_DoubleComparator());
  if (Y3.size() != Z3.size())
    return false;
  if (!std::equal(Y3.begin(), Y3.end(), Z3.begin()))
    return false;
  Z4 = std::vector<int>(X4);
  PMS11T1.sort(Z4.begin(), Z4.end(), HPCTest_IntComparator());
  if (Y4.size() != Z4.size())
    return false;
  if (!std::equal(Y4.begin(), Y4.end(), Z4.begin()))
    return false;
  Z5 = std::vector<int>(X5);
  PMS11T1.sort(Z5.begin(), Z5.end(), HPCTest_IntComparator());
  if (Y5.size() != Z5.size())
    return false;
  if (!std::equal(Y5.begin(), Y5.end(), Z5.begin()))
    return false;

  // Test 16 threads sorter
  Z1 = std::vector<int>(X1);
  PMS16T1.sort(Z1.begin(), Z1.end(), HPCTest_IntComparator());
  if (Y1.size() != Z1.size())
    return false;
  if (!std::equal(Y1.begin(), Y1.end(), Z1.begin()))
    return false;
  Z2 = std::vector<float>(X2);
  PMS16T2.sort(Z2.begin(), Z2.end(), HPCTest_FloatComparator());
  if (Y2.size() != Z2.size())
    return false;
  if (!std::equal(Y2.begin(), Y2.end(), Z2.begin()))
    return false;
  Z3 = std::vector<double>(X3);
  PMS16T3.sort(Z3.begin(), Z3.end(), HPCTest_DoubleComparator());
  if (Y3.size() != Z3.size())
    return false;
  if (!std::equal(Y3.begin(), Y3.end(), Z3.begin()))
    return false;
  Z4 = std::vector<int>(X4);
  PMS16T1.sort(Z4.begin(), Z4.end(), HPCTest_IntComparator());
  if (Y4.size() != Z4.size())
    return false;
  if (!std::equal(Y4.begin(), Y4.end(), Z4.begin()))
    return false;
  Z5 = std::vector<int>(X5);
  PMS16T1.sort(Z5.begin(), Z5.end(), HPCTest_IntComparator());
  if (Y5.size() != Z5.size())
    return false;
  if (!std::equal(Y5.begin(), Y5.end(), Z5.begin()))
    return false;

  // All checks passed
  return true;
}

}
