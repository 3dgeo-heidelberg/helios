#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"

#include <fluxionum/UnivariateNewtonRaphsonMinimizer.h>
#include <fluxionum/DesignMatrix.h>
#include <fluxionum/TemporalDesignMatrix.h>
#include <fluxionum/IndexedDesignMatrix.h>
#include <fluxionum/DiffDesignMatrix.h>
#include <fluxionum/DiffDesignMatrixInterpolator.h>
#include <fluxionum/LinearPiecesFunction.h>
#include <fluxionum/ParametricLinearPiecesFunction.h>
#include <fluxionum/FixedIterativeEulerMethod.h>
#include <fluxionum/FixedParametricIterativeEulerMethod.h>
#include <fluxionum/ClosestLesserSampleFunction.h>
#include <fluxionum/ParametricClosestLesserSampleFunction.h>

#include <armadillo>

#include <functional>
#include <cmath>

constexpr double eps = 0.00001;
std::string testDir = "data/test/";

TEST_CASE("Fluxionum: Univariate Newton-Raphson Minimization") {
    double expected = -3.0;
    UnivariateNewtonRaphsonMinimizer<double, double> unrm(
        [] (double x) -> double {return std::pow(x, 2)/2.0 + 3.0*x;},
        [] (double x) -> double {return x+3.0;},
        [] (double x) -> double {return 1;}
    );
    double x = unrm.argmin(0.0);
    REQUIRE(std::fabs(x-expected) <= eps);
}

TEST_CASE("Fluxionum: Design Matrix Building") {
    // Validation functions

    // Validate DesignMatrix
    auto validateDesignMatrix = [] (
        DesignMatrix<double> &dm,
        std::vector<std::string> const &colNames,
        arma::Mat<double> const &X
    ) -> bool {
        size_t const nRows = dm.getNumRows();
        size_t const nCols = dm.getNumColumns();
        if(X.n_rows != nRows || X.n_cols != nCols) return false;
        if(dm.hasColumnNames()){
            if(dm.getColumnNames().size() != colNames.size()) return false;
            for(size_t j = 0 ; j < nCols ; ++j){ // Check names
                if(dm.getColumnName(j) != colNames[j]) return false;
            }
        }
        else if(!colNames.empty()) return false;
        for(size_t i = 0 ; i < nRows ; ++i){ // Check values
            for(size_t j = 0 ; j < nCols ; ++j){
                if(std::fabs(dm(i, j)-X(i, j)) > eps) return false;
            }
        }
        return true;
    };

    // Validate TemporalDesignMatrix
    auto validateTemporalDesignMatrix = [&](
        TemporalDesignMatrix<double, double> &tdm,
        std::string const &timeName,
        std::vector<std::string> const &colNames,
        arma::Col<double> const &t,
        arma::Mat<double> const &X
    ) -> bool {
        if(!validateDesignMatrix(tdm, colNames, X)) return false;
        size_t const nRows = tdm.getNumRows();
        if(tdm.getTimeName() != timeName) return false;
        for(size_t i = 0 ; i < nRows ; ++i){ // Check time
            if(std::fabs(tdm[i]-t[i]) > eps) return false;
        }
        return true;
    };

    // Validate IndexedDesignMatrix
    auto validateIndexedDesignMatrix = [&](
        IndexedDesignMatrix<int, double> &idm,
        std::string const &indexName,
        std::vector<std::string> const &colNames,
        std::vector<int> const &ids,
        arma::Mat<double> const &X
    ) -> bool {
        if(!validateDesignMatrix(idm, colNames, X)) return false;
        size_t const nRows = idm.getNumRows();
        if(ids.size() != nRows) return false;
        if(idm.getIndexName() != indexName) return false;
        for(size_t i = 0 ; i < nRows ; ++i){ // Check time
            if(idm[i] != ids[i]) return false;
        }
        return true;
    };

    // Basic design matrices
    std::vector<std::string> colNamesR2({"x", "y"});
    arma::Mat<double> X1 = arma::randn(5, 2);
    DesignMatrix<double> dm1(X1, colNamesR2);
    REQUIRE(validateDesignMatrix(dm1, colNamesR2, X1));
    DesignMatrix<double> dm2(colNamesR2);
    REQUIRE(validateDesignMatrix(dm2, colNamesR2, arma::Mat<double>()));
    DesignMatrix<double> dm3;
    REQUIRE(validateDesignMatrix(dm3, std::vector<std::string>(0), arma::Mat<double>()));
    DesignMatrix<double> dm4(dm1);
    REQUIRE(validateDesignMatrix(dm4, colNamesR2, X1));
    dm4 = dm1;
    REQUIRE(validateDesignMatrix(dm4, colNamesR2, X1));

    // Temporal design matrices
    std::vector<std::string> colNamesR2t({"x", "y", "t"});
    arma::Col<double> t1 = arma::randn(5, 1);
    arma::Mat<double> X2 = arma::randn(5, 3);
    TemporalDesignMatrix<double, double> tdm1(X1, t1, "time", colNamesR2);
    REQUIRE(validateTemporalDesignMatrix(tdm1, "time", colNamesR2, t1, X1));
    TemporalDesignMatrix<double, double> tdm2(tdm1);
    REQUIRE(validateTemporalDesignMatrix(tdm2, "time", colNamesR2, t1, X1));
    tdm2 = tdm1;
    REQUIRE(validateTemporalDesignMatrix(tdm2, "time", colNamesR2, t1, X1));
    TemporalDesignMatrix<double, double> tdm3(tdm1, t1, "TIME");
    REQUIRE(validateTemporalDesignMatrix(tdm3, "TIME", colNamesR2, t1, X1));
    TemporalDesignMatrix<double, double> tdm4(tdm1, t1);
    REQUIRE(validateTemporalDesignMatrix(tdm4, "time", colNamesR2, t1, X1));
    TemporalDesignMatrix<double, double> tdm5(X2, 2, "t", colNamesR2);
    REQUIRE(validateTemporalDesignMatrix(
        tdm5, "t", colNamesR2, X2.col(2), X2.cols(0, 1)
    ));

    // Indexed design matrices
    std::vector<int> ids1({0, 1, 2, 3, 4});
    IndexedDesignMatrix<int, double> idm1(X1, ids1, "index", colNamesR2);
    REQUIRE(validateIndexedDesignMatrix(idm1, "index", colNamesR2, ids1, X1));
    IndexedDesignMatrix<int, double> idm2(idm1);
    REQUIRE(validateIndexedDesignMatrix(idm2, "index", colNamesR2, ids1, X1));
    idm2 = idm1;
    REQUIRE(validateIndexedDesignMatrix(idm2, "index", colNamesR2, ids1, X1));
    IndexedDesignMatrix<int, double> idm3(idm1, ids1, "INDEX");
    REQUIRE(validateIndexedDesignMatrix(idm3, "INDEX", colNamesR2, ids1, X1));
    IndexedDesignMatrix<int, double> idm4(idm1, ids1);
    REQUIRE(validateIndexedDesignMatrix(idm4, "index", colNamesR2, ids1, X1));
    IndexedDesignMatrix<int, double> idm5(X2, 2, "idx", colNamesR2);
    REQUIRE(validateIndexedDesignMatrix(
        idm5, "idx", colNamesR2,
        std::vector<int>({
            (int)X2(0, 2),
            (int)X2(1, 2),
            (int)X2(2, 2),
            (int)X2(3, 2),
            (int)X2(4, 2),
        }),
        X2.cols(0, 1)
    ));

    // Design matrix from file
    std::vector<std::string> hf1({"x", "y", "z"}); // Header
    arma::Mat<double> Xf1( // Matrix
        "0 0 0;"
        "0 0.1 0;"
        "0.1 0.1 0;"
        "0.1 0.2 0;"
        "0.2 0.2 0;"
        "0.2 0.2 0.1;"
        "0.2 0.3 0.1;"
        "0.3 0.3 0.1;"
        "0.3 0.3 0.2"
    );
    std::string const dmf1Path = testDir + "design_matrix_header.txt";
    DesignMatrix<double> dmf1(dmf1Path);
    REQUIRE(validateDesignMatrix(dmf1, hf1, Xf1));
    std::string const dmf2Path = testDir + "design_matrix_nonheader.txt";
    DesignMatrix<double> dmf2(dmf2Path);
    REQUIRE(validateDesignMatrix(dmf2, std::vector<std::string>(0), Xf1));

    // Temporal design matrix from file
    std::vector<string> htf1({"x", "y"}); // Header
    arma::Mat<double> Xtf1( // Matrix
        "0 0;"
        "0 0.1;"
        "0.1 0.1;"
        "0.1 0.2;"
        "0.2 0.2;"
        "0.2 0.2;"
        "0.2 0.3;"
        "0.3 0.3;"
        "0.3 0.3"
    );
    arma::Col<double> ttf1("0 0.1 0.2 0.3 0.4 0.5 0.6 0.8 1.0");
    std::string const tdmf1Path = testDir + "temporal_design_matrix_header.txt";
    TemporalDesignMatrix<double, double> tdmf1(tdmf1Path);
    REQUIRE(validateTemporalDesignMatrix(tdmf1, "t", htf1, ttf1, Xtf1));
    std::string const tdmf2Path = testDir + "temporal_design_matrix_nonheader.txt";
    TemporalDesignMatrix<double, double> tdmf2(tdmf2Path);
    REQUIRE(validateTemporalDesignMatrix(
        tdmf2, "time", std::vector<std::string>(0), ttf1, Xtf1
    ));

    // Indexed design matrix from file
    std::vector<std::string> hif1({"x", "y"}); // Header
    arma::Mat<double> Xif1(  // Matrix
        "0 0;"
        "0 0.1;"
        "0.1 0.1;"
        "0.1 0.2;"
        "0.2 0.2;"
        "0.2 0.2;"
        "0.2 0.3;"
        "0.3 0.3;"
        "0.3 0.3"
    );
    std::vector<int> iif1({0, 1, 2, 3, 4, 6, 8, 7, 5});
    std::string const idmf1Path = testDir + "indexed_design_matrix_header.txt";
    IndexedDesignMatrix<int, double> idmf1(idmf1Path);
    REQUIRE(validateIndexedDesignMatrix(idmf1, "idx", hif1, iif1, Xif1));
    std::string const idmf2Path = testDir + "indexed_design_matrix_nonheader.txt";
    IndexedDesignMatrix<int, double> idmf2(idmf2Path);
    REQUIRE(validateIndexedDesignMatrix(
        idmf2, "index", std::vector<std::string>(0), iif1, Xif1
    ));
}

TEST_CASE("Fluxionum: Design Matrix Methods") {
    // Prepare data for tests
    DesignMatrix<double> dm1(
        arma::Mat<double>(
            "0 1 2 3 4;"
            "0 1 2 3 4;"
            "0 1 2 3 4;"
        )
    );
    DesignMatrix<double> dm2(
        arma::Mat<double>(
            "5 6 7 8 9;"
            "5 6 7 8 9;"
            "5 6 7 8 9;"
        )
    );
    TemporalDesignMatrix<double, double> tdm1(
        arma::Mat<double>(
            "1 2 3;"
            "1 2 3;"
            "1 2 3;"
        ),
        arma::Col<double>("0.0 0.1 0.2")
    );
    TemporalDesignMatrix<double, double> tdm2(
        arma::Mat<double>(
            "4 5 6;"
            "4 5 6;"
            "4 5 6;"
        ),
        arma::Col<double>("0.3 0.4 0.5")
    );
    TemporalDesignMatrix<double, double> tdm3(
        arma::Mat<double>(
            "4 5 6;"
            "4 5 6;"
            "4 5 6;"
        ),
        arma::Col<double>("0.4 0.5 0.3")
    );
    TemporalDesignMatrix<double, double> tdm4(
        arma::Mat<double>(
            "-2     -2      1;"
            "0      0       1;"
            "2      2       1;"
            "4      4       1;"
            "4      4       1;"
            "4      4       1;"
            "4      4       1;"
            "4      4       1;"
            "3      3       1;"
            "2      2       1;"
            "1      1       1;"
        ),
        arma::Col<double>("-5 -4 -3 -2 -1 0 1 2 3 4 5")
    );
    IndexedDesignMatrix<int, double> idm1(
        arma::Mat<double>(
            "0.1 0.2 0.3;"
            "0.2 0.3 0.4;"
            "0.3 0.4 0.5;"
        ),
        std::vector<int>({1, 3, 5})
    );
    IndexedDesignMatrix<int, double> idm2(
        arma::Mat<double>(
            "0.4 0.5 0.6;"
            "0.5 0.6 0.7;"
            "0.7 0.8 0.9;"
        ),
        std::vector<int>({6, 8, 10})
    );

    // Validate mergeInPlace
    DesignMatrix<double> dm(dm1);
    dm.mergeInPlace(dm2);
    arma::Mat<double> em(
        "0 1 2 3 4;"
        "0 1 2 3 4;"
        "0 1 2 3 4;"
        "5 6 7 8 9;"
        "5 6 7 8 9;"
        "5 6 7 8 9;"
    );
    REQUIRE_FALSE(arma::any(arma::vectorise(arma::abs(dm.getX()-em)) > eps));
    TemporalDesignMatrix<double, double> tdm(tdm1);
    tdm.mergeInPlace(tdm2);
    em = arma::Mat<double>(
        "1 2 3;"
        "1 2 3;"
        "1 2 3;"
        "4 5 6;"
        "4 5 6;"
        "4 5 6;"
    );
    arma::Col<double> et({0.0, 0.1, 0.2, 0.3, 0.4, 0.5}); // Expected time
    REQUIRE_FALSE(arma::any(arma::vectorise(arma::abs(tdm.getX()-em)) > eps));
    REQUIRE_FALSE(arma::any((tdm.getTimeVector()-et) >eps));
    IndexedDesignMatrix<int, double> idm(idm1);
    idm.mergeInPlace(idm2);
    em = arma::Mat<double>(
        "0.1 0.2 0.3;"
        "0.2 0.3 0.4;"
        "0.3 0.4 0.5;"
        "0.4 0.5 0.6;"
        "0.5 0.6 0.7;"
        "0.7 0.8 0.9;"
    );
    std::vector<int> ei({1, 3, 5, 6, 8, 10});
    REQUIRE_FALSE(arma::any(arma::vectorise(arma::abs(idm.getX()-em)) > eps));
    for(size_t i = 0 ; i < ei.size() ; ++i){
        REQUIRE(std::fabs(ei[i]-idm.getIndices()[i]) <= eps);
    }

    // Validate swapColumns
    dm = dm1;
    dm.swapColumns(arma::uvec({0, 2, 4, 1, 3}));
    em = arma::Mat<double>(
        "0 2 4 1 3;"
        "0 2 4 1 3;"
        "0 2 4 1 3;"
    );
    REQUIRE_FALSE(arma::any(arma::vectorise(arma::abs(dm.getX()-em)) > eps));
    tdm = tdm1;
    tdm.swapColumns(arma::uvec({1, 2, 0}));
    em = arma::Mat<double>(
        "2 3 1;"
        "2 3 1;"
        "2 3 1;"
    );
    REQUIRE_FALSE(arma::any(arma::vectorise(arma::abs(tdm.getX()-em)) > eps));
    idm = idm1;
    idm.swapColumns(arma::uvec({2, 1, 0}));
    em = arma::Mat<double>(
        "0.3 0.2 0.1;"
        "0.4 0.3 0.2;"
        "0.5 0.4 0.3;"
    );
    REQUIRE_FALSE(arma::any(arma::vectorise(arma::abs(idm.getX()-em)) > eps));

    // Validate sortByTime
    tdm3.sortByTime();
    REQUIRE_FALSE(arma::any((tdm3.getTimeVector()-tdm2.getTimeVector()) > eps));

    // Validate shiftTime
    et = tdm3.getTimeVector() + 13.37;
    tdm3.shiftTime(13.37);
    REQUIRE_FALSE(arma::any((tdm3.getTimeVector()-et) > eps));

    // Validate slopeFilter
    et = arma::Col<double>("-5 -2 2 5");
    em = arma::Mat<double>(
        "-2     -2      1;"
        "4      4       1;"
        "4      4       1;"
        "1      1       1;"
    );
    REQUIRE(tdm4.slopeFilter(0.01) == 7);
    REQUIRE_FALSE(arma::any(arma::abs(tdm4.getTimeVector()-et) > eps));
    REQUIRE_FALSE(arma::any(arma::abs(arma::vectorise(tdm4.getX()-em)) > eps));
}

TEST_CASE("Fluxionum: Diff Design Matrix") {
    // Validate DiffDesignMatrix
    auto validateDiffDesignMatrix = [] (
        DiffDesignMatrix<double, double> &ddm,
        std::string const &timeName,
        std::vector<std::string> const &colNames,
        arma::Col<double> const &t,
        arma::Mat<double> const &A,
        DiffDesignMatrixType diffType
    ) -> bool {
        if(ddm.getA().n_rows != ddm.getTimeVector().n_rows) return false;
        size_t const nRows = ddm.getNumRows();
        size_t const nCols = ddm.getNumColumns();
        if(A.n_rows != nRows || A.n_cols != nCols) return false;
        if(ddm.hasColumnNames()){
            if(ddm.getColumnNames().size() != colNames.size()) return false;
            for(size_t j = 0 ; j < nCols ; ++j){ // Check names
                if(ddm.getColumnName(j) != colNames[j]) return false;
            }
        }
        else if(!colNames.empty()) return false;
        for(size_t i = 0 ; i < nRows ; ++i){
            for(size_t j = 0 ; j < nCols ; ++j){
                if(std::fabs(ddm(i, j)-A(i, j)) > eps) return false;
            }
        }
        if(diffType != ddm.getDiffType()) return false;
        if(ddm.getTimeName() != timeName) return false;
        for(size_t i = 0 ; i < nRows ; ++i){ // Check time
            if(std::fabs(ddm[i]-t[i]) > eps) return false;
        }
        return true;
    };

    arma::Col<double> Et1("-3 -2 -1 0 1 2");
    arma::Col<double> Et3("-2.5 -1.5 -0.5 0.5 1.5");
    arma::Mat<double> EA1(
        "0.1 -0.5;"
        "0.1 -0.3;"
        "0.1 -0.1;"
        "0.1 0.1;"
        "0.1 0.3;"
        "0.1 0.5"
    );
    arma::Mat<double> EA3(
        "0.1 -0.4;"
        "0.1 -0.2;"
        "0.1 0.0;"
        "0.1 0.2;"
        "0.1 0.4;"
    );
    std::vector<std::string> EcolNames1({"f1", "f2"});
    std::string EtimeName1 = "t";
    std::string EtimeName2 = "time";

    std::vector<std::string> colNames1({"t", "f1", "f2"});
    arma::Mat<double> X1 = arma::Mat<double>(
        "-3 -0.3 0.9;"
        "-2 -0.2 0.4;"
        "-1 -0.1 0.1;"
        "0 0 0;"
        "1 0.1 0.1;"
        "2 0.2 0.4;"
        "3 0.3 0.9"
    );
    arma::Mat<double> X2 = arma::Mat<double>(
        "3 0.3 0.9;"
        "-1 -0.1 0.1;"
        "0 0 0;"
        "-3 -0.3 0.9;"
        "1 0.1 0.1;"
        "-2 -0.2 0.4;"
        "2 0.2 0.4"
    );
    DesignMatrix<double> dm1(X1, colNames1); // time sorted
    DesignMatrix<double> dm2(X2); // non time sorted

    // Build temporal design matrix
    TemporalDesignMatrix<double, double> tdm1(dm1, 0); // time sorted
    TemporalDesignMatrix<double, double> tdm2(dm2, 0); // non time sorted

    // Build diff design matrix from previous temporal design matrix
    DiffDesignMatrix<double, double> ddm1( // built from time sorted
        tdm1, DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES
    );
    DiffDesignMatrix<double, double> ddm2( // build from non time sorted
        tdm2, DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES
    );
    DiffDesignMatrix<double, double> ddm3( // build from non time sorted
        tdm2, DiffDesignMatrixType::CENTRAL_FINITE_DIFFERENCES
    );

    // Validate built diff design matrix
    REQUIRE(validateDiffDesignMatrix(
        ddm1, EtimeName1, EcolNames1, Et1, EA1,
        DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES
    ));
    REQUIRE(validateDiffDesignMatrix(
        ddm2, EtimeName2, std::vector<std::string>(0), Et1, EA1,
        DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES
    ));
    REQUIRE(validateDiffDesignMatrix(
        ddm3, EtimeName2, std::vector<std::string>(0), Et3, EA3,
        DiffDesignMatrixType::CENTRAL_FINITE_DIFFERENCES
    ));

    // Load diff design matrix from file
    std::string const ddmf1Path = testDir + "diff_design_matrix_header.txt";
    DiffDesignMatrix<double, double> ddmf1(ddmf1Path);
    REQUIRE(validateDiffDesignMatrix(
        ddmf1, EtimeName1, EcolNames1, Et1, EA1,
        DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES
    ));
    std::string const ddmf2Path = testDir + "diff_design_matrix_nonheader.txt";
    DiffDesignMatrix<double, double> ddmf2(ddmf2Path);
    REQUIRE(validateDiffDesignMatrix(
        ddmf2, EtimeName2, std::vector<std::string>(0), Et3, EA3,
        DiffDesignMatrixType::CENTRAL_FINITE_DIFFERENCES
    ));
}

TEST_CASE("Fluxionum: Design Functions") {
    TemporalDesignMatrix<double, double> tdm1(
        arma::Mat<double>(
            "0 0;"
            "2 2;"
            "4 3;"
            "6 3;"
            "9 4;"
        ),
        0
    );
    TemporalDesignMatrix<double, double> tdm2(
        arma::Mat<double>(
            "0 0 3;"
            "2 2 4;"
            "4 3 3;"
            "6 3 6;"
            "9 4 5;"
        ),
        0
    );
    DiffDesignMatrix<double, double> ffd1 = tdm1.toDiffDesignMatrix(
        DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES
    );
    DiffDesignMatrix<double, double> ffd2 = tdm2.toDiffDesignMatrix(
        DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES
    );

    // Validate LinearPiecesFunction
    arma::Col<double> intercept(
        tdm1.getColumnCopy(0).subvec(0,tdm1.getNumRows()-2)
    );
    arma::Col<double> slope = ffd1.getA().col(0);
    LinearPiecesFunction<double, double> lpf1 =
        DiffDesignMatrixInterpolator::makeLinearPiecesFunction(
            ffd1,
            slope,
            intercept
        );
    LinearPiecesFunction<double, double> lpf2 =
        DiffDesignMatrixInterpolator::makeLinearPiecesFunction(
            ffd1, tdm1, 0, &intercept, &slope
        );
    arma::Col<double> lpf1t("-1 0 1 2 3 4 5 6 7.5 9");
    arma::Col<double> lpf1E("-1 0 1 2 2.5 3 3 3 3.5 4");
    for(size_t i = 0 ; i < lpf1t.n_elem ; ++i){
        REQUIRE(std::fabs(lpf1(lpf1t.at(i)) - lpf1E.at(i)) <= eps);
        REQUIRE(std::fabs(lpf2(lpf1t.at(i)) - lpf1E.at(i)) <= eps);
    }

    // Validate ParametricLinearPiecesFunction
    ParametricLinearPiecesFunction<double, double> plpf1 =
        DiffDesignMatrixInterpolator::makeParametricLinearPiecesFunction(
            ffd2,
            tdm2.getX()
        );
    ParametricLinearPiecesFunction<double, double> plpf2 =
        DiffDesignMatrixInterpolator::makeParametricLinearPiecesFunction(
            ffd2, tdm2
        );
    arma::Mat<double> plpf1E(
        "-1 2.5;"
        "0 3;"
        "1 3.5;"
        "2 4;"
        "2.5 3.5;"
        "3 3;"
        "3 4.5;"
        "3 6;"
        "3.5 5.5;"
        "4 5"
    );
    for(size_t i = 0 ; i < lpf1t.n_elem ; ++i){
        arma::Col<double> plpf1y = plpf1(lpf1t.at(i));
        arma::Col<double> plpf2y = plpf2(lpf1t.at(i));
        for(size_t j = 0 ; j < 2 ; ++j){
            REQUIRE(std::fabs(plpf1y.at(j) - plpf1E.at(i, j)) <= eps);
            REQUIRE(std::fabs(plpf2y.at(j) - plpf1E.at(i, j)) <= eps);
        }
    }

    // Validate FixedIterativeEulerMethod
    arma::Col<double> fiem1t("0 1 2 3 4 5 6 7.5 9");
    arma::Col<double> fiem1E("0 1 2 2.5 3 3 3 3.5 4");
    arma::Col<double> ySamples1(
        tdm1.getColumnCopy(0).subvec(0, tdm1.getNumRows()-2)
    );
    arma::Col<double> ySamples2;
    arma::Col<double> dydtSamples1(
        ffd1.getA().col(0)
    );
    arma::Col<double> dydtSamples2;
    ClosestLesserSampleFunction<double, double> clsf1(
        ffd1.getTimeVector(),
        dydtSamples1,
        0
    );
    ClosestLesserSampleFunction<double, double> clsf2 = clsf1;
    FixedIterativeEulerMethod<double, double> fiem1 =
        DiffDesignMatrixInterpolator::makeFixedIterativeEulerMethod(
            ffd1,
            ySamples1,
            clsf1
        );
    FixedIterativeEulerMethod<double, double> fiem2 =
        DiffDesignMatrixInterpolator::makeFixedIterativeEulerMethod(
            ffd1,
            tdm1,
            0,
            &ySamples2,
            &clsf2,
            &dydtSamples2
        );
    for(size_t i = 0 ; i < fiem1t.n_elem ; ++i){
        double const h = (i==0) ? 0 : fiem1t.at(i) - fiem1t.at(i-1);
        REQUIRE(std::fabs(fiem1(h) - fiem1E.at(i)) <= eps);
        REQUIRE(std::fabs(fiem2(h) - fiem1E.at(i)) <= eps);
    }

    // Validate ParametricFixedIterativeEulerMethod
    arma::Mat<double> pfiem1E(
        "0 3;"
        "1 3.5;"
        "2 4;"
        "2.5 3.5;"
        "3 3;"
        "3 4.5;"
        "3 6;"
        "3.5 5.5;"
        "4 5"
    );
    arma::Mat<double> const &pySamples1 = tdm2.getX();
    arma::Mat<double> const &pdydtSamples1 = ffd2.getA();
    ParametricClosestLesserSampleFunction<double, double> pclsf1(
        ffd2.getTimeVector(),
        pdydtSamples1,
        0
    );
    FixedParametricIterativeEulerMethod<double, double> fpiem1 =
        DiffDesignMatrixInterpolator::makeFixedParametricIterativeEulerMethod(
            ffd2,
            pySamples1,
            pclsf1
        );
    ParametricClosestLesserSampleFunction<double, double> pclfs2(pclsf1);
    FixedParametricIterativeEulerMethod<double, double> fpiem2 =
        DiffDesignMatrixInterpolator::makeFixedParametricIterativeEulerMethod(
            ffd2,
            tdm2,
            &pclfs2
        );

    for(size_t i = 0 ; i < fiem1t.n_elem ; ++i){
        double const h = (i==0) ? 0 : fiem1t.at(i) - fiem1t.at(i-1);
        REQUIRE(arma::approx_equal(fpiem1(h), pfiem1E.row(i).as_col(), "absdiff", eps));
        REQUIRE(arma::approx_equal(fpiem2(h), pfiem1E.row(i).as_col(), "absdiff", eps));
    }
}