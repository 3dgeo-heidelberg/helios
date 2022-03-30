#pragma once

#include <fluxionum/UnivariateNewtonRaphsonMinimizer.h>
#include <fluxionum/DesignMatrix.h>
#include <fluxionum/TemporalDesignMatrix.h>
#include <fluxionum/IndexedDesignMatrix.h>
#include <fluxionum/DiffDesignMatrix.h>

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
};

// ***  R U N  *** //
// *************** //
bool FluxionumTest::run(){
    // Run tests
    if(!testUnivariateNewtonRaphsonMinimization()) return false;
    if(!testDesignMatrixBuilding()) return false;
    //if(!testDiffDesignMatrix()) return false; // TODO Rethink : Implement
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
        DesignMatrix<double> &, vector<string>, arma::Mat<double>
    )> validateDesignMatrix = [&] (
            DesignMatrix<double> &dm,
            vector<string> colNames,
            arma::Mat<double> X
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
                if(std::fabs(dm(i, j)-X(i, j)) > 0.00001) return false;
            }
        }
        return true;
    };
    std::function<bool( // Validate TemporalDesignMatrix
        TemporalDesignMatrix<double, double> &, string, vector<string>,
        arma::Col<double>, arma::Mat<double>
    )> validateTemporalDesignMatrix = [&](
            TemporalDesignMatrix<double, double> &tdm,
            string timeName,
            vector<string> colNames,
            arma::Col<double> t,
            arma::Mat<double> X
    ) -> bool
    {
        if(!validateDesignMatrix(tdm, colNames, X)) return false;
        size_t const nRows = tdm.getNumRows();
        if(tdm.getTimeName() != timeName) return false;
        for(size_t i = 0 ; i < nRows ; ++i){ // Check time
            if(std::fabs(tdm[i]-t[i]) > 0.00001) return false;
        }
        return true;
    };
    std::function<bool( // Validate IndexedDesignMatrix
        IndexedDesignMatrix<int, double> &, string, vector<string>,
        vector<int>, arma::Mat<double>
    )> validateIndexedDesignMatrix = [&](
        IndexedDesignMatrix<int, double> &idm,
        string indexName,
        vector<string> colNames,
        vector<int> ids,
        arma::Mat<double> X
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
    // TODO Rethink : Test header
    // TODO Rethink : Test nonheader

    // On passed return true
    // TODO Rethink : Valgrind
    return true;
}

}