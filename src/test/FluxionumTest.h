#pragma once

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

namespace HeliosTests{

using namespace fluxionum;
using std::function;

/**
 * @author Alberto M. Esmoris Pena
 * @verison 1.0
 * @brief Fluxionum test
 */
class FluxionumTest : public BaseTest{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Decimal precision for validation purposes
     */
    double const eps = 0.00001;
    /**
     * @brief The directory where test files are located
     */
    std::string testDir;

    // ***  CONSTRUCTOR  *** //
    // ********************* //
    /**
     * @brief Fluxionum test constructor
     */
    FluxionumTest(std::string testDir) :
        BaseTest("Fluxionum test"),
        testDir(testDir)
    {}

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseTest::run
     */
    bool run() override;

    // ***  SUB-TESTS  *** //
    // ******************* //
    /**
     * @brief Test univariate Newton-Raphson minimization
     * @return True if passed, false otherwise
     */
    bool testUnivariateNewtonRaphsonMinimization();
    /**
     * @brief Test the building of different design matrices
     * @return True if passed, false otherwise
     */
    bool testDesignMatrixBuilding();
    /**
     * @brief Test the generation of differential design matrices
     * @return True if passed, false otherwise
     */
    bool testDiffDesignMatrix();
    /**
     * @brief Test functions from design matrices
     * @return True if passed, false otherwise
     */
    bool testDesignFunctions();
};

// ***  R U N  *** //
// *************** //
bool FluxionumTest::run(){
    // Run tests
    if(!testUnivariateNewtonRaphsonMinimization()) return false;
    if(!testDesignMatrixBuilding()) return false;
    if(!testDiffDesignMatrix()) return false;
    if(!testDesignFunctions()) return false;
    return true;
}

// ***  SUB-TESTS  *** //
// ******************* //
bool FluxionumTest::testUnivariateNewtonRaphsonMinimization(){
    double expected = -3.0;
    UnivariateNewtonRaphsonMinimizer<double, double> unrm(
        [] (double x) -> double {return std::pow(x, 2)/2.0 + 3.0*x;},
        [] (double x) -> double {return x+3.0;},
        [] (double x) -> double {return 1;}
    );
    double x = unrm.argmin(0.0);
    return std::fabs(x-expected) <= eps;
}

bool FluxionumTest::testDesignMatrixBuilding(){
    // Validation functions
    std::function<bool( // Validate DesignMatrix
        DesignMatrix<double> &, vector<string> const &,
        arma::Mat<double> const &
    )> validateDesignMatrix = [&] (
            DesignMatrix<double> &dm,
            vector<string> const &colNames,
            arma::Mat<double> const &X
        ) -> bool
    {
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
    std::function<bool( // Validate TemporalDesignMatrix
        TemporalDesignMatrix<double, double> &, string const &,
        vector<string> const &,
        arma::Col<double> const &, arma::Mat<double> const &
    )> validateTemporalDesignMatrix = [&](
            TemporalDesignMatrix<double, double> &tdm,
            string const &timeName,
            vector<string> const &colNames,
            arma::Col<double> const &t,
            arma::Mat<double> const &X
    ) -> bool
    {
        if(!validateDesignMatrix(tdm, colNames, X)) return false;
        size_t const nRows = tdm.getNumRows();
        if(tdm.getTimeName() != timeName) return false;
        for(size_t i = 0 ; i < nRows ; ++i){ // Check time
            if(std::fabs(tdm[i]-t[i]) > eps) return false;
        }
        return true;
    };
    std::function<bool( // Validate IndexedDesignMatrix
        IndexedDesignMatrix<int, double> &, string const&,
        vector<string> const&,
        vector<int> const&, arma::Mat<double> const&
    )> validateIndexedDesignMatrix = [&](
        IndexedDesignMatrix<int, double> &idm,
        string const &indexName,
        vector<string> const &colNames,
        vector<int> const &ids,
        arma::Mat<double> const &X
    ) -> bool
    {
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
    vector<string> colNamesR2({"x", "y"});
    arma::Mat<double> X1 = arma::randn(5, 2);
    DesignMatrix<double> dm1(X1, colNamesR2);
    if(!validateDesignMatrix(dm1, colNamesR2, X1)) return false;
    DesignMatrix<double> dm2(colNamesR2);
    if(!validateDesignMatrix(dm2, colNamesR2, arma::Mat<double>()))
        return false;
    DesignMatrix<double> dm3;
    if(!validateDesignMatrix(dm3, vector<string>(0), arma::Mat<double>()))
        return false;
    DesignMatrix<double> dm4(dm1);
    if(!validateDesignMatrix(dm4, colNamesR2, X1)) return false;
    dm4 = dm1;
    if(!validateDesignMatrix(dm4, colNamesR2, X1)) return false;

    // Temporal design matrices
    vector<string> colNamesR2t({"x", "y", "t"});
    arma::Col<double> t1 = arma::randn(5, 1);
    arma::Mat<double> X2 = arma::randn(5, 3);
    TemporalDesignMatrix<double, double> tdm1(X1, t1, "time", colNamesR2);
    if(!validateTemporalDesignMatrix(tdm1, "time", colNamesR2, t1, X1))
        return false;
    TemporalDesignMatrix<double, double> tdm2(tdm1);
    if(!validateTemporalDesignMatrix(tdm2, "time", colNamesR2, t1, X1))
        return false;
    tdm2 = tdm1;
    if(!validateTemporalDesignMatrix(tdm2, "time", colNamesR2, t1, X1))
        return false;
    TemporalDesignMatrix<double, double> tdm3(tdm1, t1, "TIME");
    if(!validateTemporalDesignMatrix(tdm3, "TIME", colNamesR2, t1, X1))
        return false;
    TemporalDesignMatrix<double, double> tdm4(tdm1, t1);
    if(!validateTemporalDesignMatrix(tdm4, "time", colNamesR2, t1, X1))
        return false;
    TemporalDesignMatrix<double, double> tdm5(X2, 2, "t", colNamesR2);
    if(!validateTemporalDesignMatrix(
        tdm5, "t", colNamesR2, X2.col(2), X2.cols(0, 1)
    )) return false;

    // Indexed design matrices
    vector<int> ids1({0, 1, 2, 3, 4});
    IndexedDesignMatrix<int, double> idm1(X1, ids1, "index", colNamesR2);
    if(!validateIndexedDesignMatrix(idm1, "index", colNamesR2, ids1, X1))
        return false;
    IndexedDesignMatrix<int, double> idm2(idm1);
    if(!validateIndexedDesignMatrix(idm2, "index", colNamesR2, ids1, X1))
        return false;
    idm2 = idm1;
    if(!validateIndexedDesignMatrix(idm2, "index", colNamesR2, ids1, X1))
        return false;
    IndexedDesignMatrix<int, double> idm3(idm1, ids1, "INDEX");
    if(!validateIndexedDesignMatrix(idm3, "INDEX", colNamesR2, ids1, X1))
        return false;
    IndexedDesignMatrix<int, double> idm4(idm1, ids1);
    if(!validateIndexedDesignMatrix(idm4, "index", colNamesR2, ids1, X1))
        return false;
    IndexedDesignMatrix<int, double> idm5(X2, 2, "idx", colNamesR2);
    if(!validateIndexedDesignMatrix(
        idm5, "idx", colNamesR2,
        vector<int>({
            (int)X2(0, 2),
            (int)X2(1, 2),
            (int)X2(2, 2),
            (int)X2(3, 2),
            (int)X2(4, 2),
        }),
        X2.cols(0, 1)
    )) return false;

    // Design matrix from file
    vector<string> hf1({"x", "y", "z"}); // Header
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
    string const dmf1Path = testDir + "design_matrix_header.txt";
    DesignMatrix<double> dmf1(dmf1Path);
    if(!validateDesignMatrix(dmf1, hf1, Xf1)) return false;
    string const dmf2Path = testDir + "design_matrix_nonheader.txt";
    DesignMatrix<double> dmf2(dmf2Path);
    if(!validateDesignMatrix(dmf2, vector<string>(0), Xf1)) return false;

    // Temporal design matrix from file
    vector<string> htf1({"x", "y"}); // Header
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
    string const tdmf1Path = testDir + "temporal_design_matrix_header.txt";
    TemporalDesignMatrix<double, double> tdmf1(tdmf1Path);
    if(!validateTemporalDesignMatrix(tdmf1, "t", htf1, ttf1, Xtf1))
        return false;
    string const tdmf2Path = testDir + "temporal_design_matrix_nonheader.txt";
    TemporalDesignMatrix<double, double> tdmf2(tdmf2Path);
    if(!validateTemporalDesignMatrix(
        tdmf2, "time", vector<string>(0), ttf1, Xtf1
    )) return false;

    // Indexed design matrix from file
    vector<string> hif1({"x", "y"}); // Header
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
    vector<int> iif1({0, 1, 2, 3, 4, 6, 8, 7, 5});
    string const idmf1Path = testDir + "indexed_design_matrix_header.txt";
    IndexedDesignMatrix<int, double> idmf1(idmf1Path);
    if(!validateIndexedDesignMatrix(idmf1, "idx", hif1, iif1, Xif1))
        return false;
    string const idmf2Path = testDir + "indexed_design_matrix_nonheader.txt";
    IndexedDesignMatrix<int, double> idmf2(idmf2Path);
    if(!validateIndexedDesignMatrix(
        idmf2, "index", vector<string>(0), iif1, Xif1
    )) return false;


    // On passed return true
    return true;
}

bool FluxionumTest::testDiffDesignMatrix(){
    // Validation function
    std::function<bool( // Validate DiffDesignMatrix
        DiffDesignMatrix<double, double> &, string const&,
        vector<string> const&,
        arma::Col<double> const&, arma::Mat<double> const&,
        DiffDesignMatrixType
    )> validateDiffDesignMatrix = [&] (
            DiffDesignMatrix<double, double> &ddm,
            string const &timeName,
            vector<string> const &colNames,
            arma::Col<double> const &t,
            arma::Mat<double> const &A,
            DiffDesignMatrixType diffType
        ) -> bool
    {
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


    // Expected diff design matrices
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
    vector<string> EcolNames1({"f1", "f2"});
    string EtimeName1 = "t";
    string EtimeName2 = "time";

    // Build design matrix
    vector<string> colNames1({"t", "f1", "f2"});
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
    DiffDesignMatrix<double, double> ddm2( // built from non time sorted
        tdm2, DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES
    );
    DiffDesignMatrix<double, double> ddm3( // build from non time sorted
        tdm2, DiffDesignMatrixType::CENTRAL_FINITE_DIFFERENCES
    );

    // Validate built diff design matrix
    if(!validateDiffDesignMatrix(
        ddm1, EtimeName1, EcolNames1, Et1, EA1,
        DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES
    ))
        return false;
    if(!validateDiffDesignMatrix(
        ddm2, EtimeName2, vector<string>(0), Et1, EA1,
        DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES
    )) return false;
    if(!validateDiffDesignMatrix(
        ddm3, EtimeName2, vector<string>(0), Et3, EA3,
        DiffDesignMatrixType::CENTRAL_FINITE_DIFFERENCES
    )) return false;

    // Load diff design matrix from file
    string const ddmf1Path = testDir + "diff_design_matrix_header.txt";
    DiffDesignMatrix<double, double> ddmf1(ddmf1Path);
    if(!validateDiffDesignMatrix(
        ddmf1, EtimeName1, EcolNames1, Et1, EA1,
        DiffDesignMatrixType::FORWARD_FINITE_DIFFERENCES
    )) return false;
    string const ddmf2Path = testDir + "diff_design_matrix_nonheader.txt";
    DiffDesignMatrix<double, double> ddmf2(ddmf2Path);
    if(!validateDiffDesignMatrix(
        ddmf2, EtimeName2, vector<string>(0), Et3, EA3,
        DiffDesignMatrixType::CENTRAL_FINITE_DIFFERENCES
    )) return false;

    // On passed return true
    return true;
}

bool FluxionumTest::testDesignFunctions(){
    // Prepare data
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
    LinearPiecesFunction<double, double> lpf1 = \
    DiffDesignMatrixInterpolator::makeLinearPiecesFunction(
        ffd1,
        arma::Col<double>(tdm1.getColumnCopy(0).subvec(0,tdm1.getNumRows()-2)),
        0
    );
    LinearPiecesFunction<double, double> lpf2 = \
    DiffDesignMatrixInterpolator::makeLinearPiecesFunction(ffd1, tdm1);
    arma::Col<double> lpf1t("-1 0 1 2 3 4 5 6 7.5 9");
    arma::Col<double> lpf1E("-1 0 1 2 2.5 3 3 3 3.5 4");
    for(size_t i = 0 ; i < lpf1t.n_elem ; ++i){
        if(std::fabs(lpf1(lpf1t.at(i)) - lpf1E.at(i)) > eps) return false;
        if(std::fabs(lpf2(lpf1t.at(i)) - lpf1E.at(i)) > eps) return false;
    }

    // Validate ParametricLinearPiecesFunction
    ParametricLinearPiecesFunction<double, double> plpf1 = \
    DiffDesignMatrixInterpolator::makeParametricLinearPiecesFunction(
        ffd2,
        arma::Mat<double>(tdm2.getX().rows(0, tdm2.getNumRows()-2))
    );
    ParametricLinearPiecesFunction<double, double> plpf2 = \
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
            if(std::fabs(plpf1y.at(j) - plpf1E.at(i, j)) > eps) return false;
            if(std::fabs(plpf2y.at(j) - plpf1E.at(i, j)) > eps) return false;
        }
    }

    // Validate FixedIterativeEulerMethod
    arma::Col<double> fiem1t("0 1 2 3 4 5 6 7.5 9");
    arma::Col<double> fiem1E("0 1 2 2.5 3 3 3 3.5 4");
    arma::Col<double> ySamples1(
        tdm1.getColumnCopy(0).subvec(0, tdm1.getNumRows()-2)
    );
    arma::Col<double> ySamples2;
    arma::Col<double> dydtSamples1( // Vector of known derivative samples
        ffd1.getA().col(0)
    );
    arma::Col<double> dydtSamples2;
    ClosestLesserSampleFunction<double, double> clsf1(
        ffd1.getTimeVector(),
        dydtSamples1,
        0
    );
    ClosestLesserSampleFunction<double, double> clsf2 = clsf1;
    FixedIterativeEulerMethod<double, double> fiem1 = \
    DiffDesignMatrixInterpolator::makeFixedIterativeEulerMethod(
        ffd1,
        ySamples1,
        clsf1
    );
    FixedIterativeEulerMethod<double, double> fiem2 = \
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
        if(std::fabs(fiem1(h) - fiem1E.at(i)) > eps) return false;
        if(std::fabs(fiem2(h) - fiem1E.at(i)) > eps) return false;
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
    FixedParametricIterativeEulerMethod<double, double> fpiem1 = \
    DiffDesignMatrixInterpolator::makeFixedParametricIterativeEulerMethod(
        ffd2,
        pySamples1,
        pclsf1
    );
    ParametricClosestLesserSampleFunction<double, double> pclfs2(pclsf1);
    FixedParametricIterativeEulerMethod<double, double> fpiem2 = \
    DiffDesignMatrixInterpolator::makeFixedParametricIterativeEulerMethod(
        ffd2,
        tdm2,
        &pclfs2
    );

    for(size_t i = 0 ; i < fiem1t.n_elem ; ++i){
        double const h = (i==0) ? 0 : fiem1t.at(i) - fiem1t.at(i-1);
        if(arma::any(arma::abs(fpiem1(h) - pfiem1E.row(i).as_col()) > eps))
            return false;
        if(arma::any(arma::abs(fpiem2(h) - pfiem1E.row(i).as_col()) > eps))
            return false;
    }

    // On passed return true
    return true;
}

}