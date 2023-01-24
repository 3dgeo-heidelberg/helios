#pragma once

#include <BaseTest.h>
#include <adt/exprtree/UnivarExprTreeNode.h>
#include <adt/exprtree/UnivarExprTreeStringFactory.h>

#include <memory>
#include <vector>

namespace HeliosTests{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Expression tree test
 */
class ExprTreeTest : public BaseTest{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Decimal precision for validation purposes
     */
    double eps = 0.00001; // Decimal precision for validation purposes

    // ***  CONSTRUCTOR *** //
    // ******************** //
    /**
     * @brief Expression tree test constructor
     */
    ExprTreeTest() : BaseTest("Expression tree test"){}

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseTest::run
     */
    bool run() override;

    // ***  SUB-TESTS  *** //
    // ******************* //
    /**
     * @brief Test univariate expression tree
     * @return True if passed, false otherwise
     */
    bool testUnivarExprTree();

    // ***  INNER METHODS  *** //
    // *********************** //
    /**
     * @brief Validate \f$f(t) \approx y\f$
     * @return True if the test is passed, false otherwise
     */
    template <typename InputType, typename OutputType>
    bool validate(
        std::shared_ptr<IExprTreeNode<InputType, OutputType>> f,
        std::vector<InputType> const &t,
        std::vector<OutputType> const &y
    ){
        std::vector<OutputType> ft;
        for(InputType const & ti : t) ft.push_back(f->eval(ti));
        size_t const m = ft.size();
        size_t const n = y.size();
        if(m!=n) return false;
        for(size_t i = 0 ; i < m ; ++i){
            double const absDiff = std::fabs(ft[i]-y[i]);
            if(absDiff > eps) return false;
        }
        return true;
    }
};

// ***  R U N  *** //
// *************** //
bool ExprTreeTest::run(){
    bool passed = true;
    passed = passed && testUnivarExprTree();
    return passed;
}


// ***  SUB-TESTS  *** //
// ******************* //
bool ExprTreeTest::testUnivarExprTree(){
    // Prepare factory
    UnivarExprTreeStringFactory<double> uetsf;

    // Test case 1: 1+(t-1)^2
    std::shared_ptr<IExprTreeNode<double, double>> node = uetsf.makeShared(
        "1+(t-1)^2"
    );
    std::vector<double> t({
        -10, -3.14, -2.72, -1.1, -0.34, 0, 0.34, 1.1, 2.72, 3.14, 10
    });
    std::vector<double> y({
        122.00000000,  18.13960000,  14.83840000,   5.41000000,
        2.79560000,   2.00000000,   1.43560000,   1.01000000,
        3.95840000,   5.57960000,  82.00000000
    });
    if(!validate<double, double>(node, t, y)) return false;

    // TODO Restore : Commented tests below
    // Test case 2: exp(t)+3*cos(1+2*t)
    node = uetsf.makeShared("exp(t)+3*cos(1+2*t)");
    y = std::vector<double>({
        2.96615925e+00,  1.65614048e+00, -7.41224555e-01,  1.41994435e+00,
        3.55947658e+00,  2.62090692e+00,  1.07798733e+00,  9.28169656e-03,
        1.81435115e+01,  2.47328066e+01,  2.20248226e+04
    });
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 3: ln(t/2)*sin(t)
    node = uetsf.makeShared("ln(t/2)*sin(t)");
    y = std::vector<double>({
        NAN,             NAN,                           NAN,             NAN,
        NAN,             NAN,               -5.90924735e-01, -5.32796735e-01,
        1.25827096e-01,  7.18406901e-04,    -8.75568201e-01
    });
    if(!validate<double, double>(node, t, y)) return false;



    // Test case 4: acos(1/3)+t/pi
    node = uetsf.makeShared("acos(1/3)+t/pi");
    y = std::vector<double>({
        -1.95213944,  0.23146637,  0.36515653,  0.88081854,  1.12273406,
        1.23095942,  1.33918478,  1.58110029,  2.09676231,  2.23045246,
        4.41405828
    });
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 5: 5*t
    node = uetsf.makeShared("5*t");
    y = std::vector<double>({
        -50.00000000, -15.70000000, -13.60000000,  -5.50000000,
        -1.70000000,   0.00000000,   1.70000000,   5.50000000,
        13.60000000,  15.70000000,  50.00000000
    });
    if(!validate<double, double>(node, t, y)) return false;


    // Test case 6: t
    node = uetsf.makeShared("t");
    y = std::vector<double>({
        -10.00000000,  -3.14000000,  -2.72000000,  -1.10000000,
        -0.34000000,   0.00000000,   0.34000000,   1.10000000,
        2.72000000,   3.14000000,  10.00000000
    });
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 7: 7
    node = uetsf.makeShared("7");
    y = std::vector<double>({7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7});
    validate<double, double>(node, t, y);

    // Test case 8: 4*t^3 + 3*t^2 + 2*t - 1
    node = uetsf.makeShared("4*t^3 + 3*t^2 + 2*t - 1");
    y = std::vector<double>({
        -3.72100000e+03, -1.01537776e+02, -6.47393920e+01, -4.89400000e+00,
        -1.49041600e+00, -1.00000000e+00,  1.84016000e-01,  1.01540000e+01,
        1.07129792e+02,  1.58695376e+02,  4.31900000e+03
    });
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 9: 3600 - t^(t+1)
    node = uetsf.makeShared("3600 - t^(t+1)");
    y = std::vector<double>({
        3.60000000e+03,             NAN,             NAN,             NAN,
        NAN,  3.60000000e+03,  3.59976440e+03,  3.59877841e+03,
        3.55863850e+03,  3.48589919e+03, -9.99999964e+10
    });
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 10: cos(t)
    node = uetsf.makeShared("cos(t)");
    y = std::vector<double>({
        -0.83907153, -0.99999873, -0.91243836,  0.45359612,  0.94275467,
        1.00000000,  0.94275467,  0.45359612, -0.91243836, -0.99999873,
        -0.83907153
    });
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 11: sin(t^2 + 3)
    node = uetsf.makeShared("sin(t^2 + 3)");
    y = std::vector<double>({
        0.62298863,  0.28904527, -0.82692784, -0.87643472,  0.02598973,
        0.14112001,  0.02598973, -0.87643472, -0.82692784,  0.28904527,
        0.62298863
    });
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 12: 3 + 5 * 7 + 4
    node = uetsf.makeShared("3.1 + 5.0 * 7 +4.4");
    y = std::vector<double>({
        42.5, 42.5, 42.5, 42.5, 42.5, 42.5, 42.5, 42.5, 42.5, 42.5, 42.5
    });
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 13: -t
    node = uetsf.makeShared("-t");
    y = std::vector<double>({
        10.00000000,   3.14000000,   2.72000000,   1.10000000,
        0.34000000,  -0.00000000,  -0.34000000,  -1.10000000,
        -2.72000000,  -3.14000000, -10.00000000
    });
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 14: -1
    node = uetsf.makeShared("-1");
    y = std::vector<double>({-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1});
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 15: -t*7
    node = uetsf.makeShared("-t*7");
    y = std::vector<double>({
        70.00000000,  21.98000000,  19.04000000,   7.70000000,
        2.38000000,  -0.00000000,  -2.38000000,  -7.70000000,
        -19.04000000, -21.98000000, -70.00000000
    });
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 16: -1+7
    node = uetsf.makeShared("-1 +7");
    y = std::vector<double>({6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6});
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 17: ((t-1)*((1+t) / 2))/(t+0.01)^2
    node = uetsf.makeShared("((t-1)*((1+t) / 2))/(t+0.01)^2");
    y = std::vector<double>({
        4.95991487e-01,  4.52163439e-01,  4.35614983e-01,  8.83763993e-02,
        -4.06060606e+00, -5.00000000e+03, -3.60979592e+00,  8.52203555e-02,
        4.29255726e-01,  4.46439909e-01,  4.94011483e-01
    });
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 18: (pi/3)^(abs(t)^(e-1))
    node = uetsf.makeShared("(pi/3)^(abs(t)^(e-1))");
    y = std::vector<double>({
        11.14208743,  1.39014251,  1.29353943,  1.05582653,  1.00725079,
        1.00000000,  1.00725079,  1.05582653,  1.29353943,  1.39014251,
        11.14208743
    });
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 19: atan2(t, 1) + pi
    node = uetsf.makeShared("atan2(t, 1) +pi");
    y = std::vector<double>({
        1.67046498, 1.87911199, 1.92310505, 2.30861139, 2.81385415,
        3.14159265, 3.46933116, 3.97457392, 4.36008026, 4.40407332,
        4.61272033
    });
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 20: t-(-t) = 2*t
    node = uetsf.makeShared("t-(-t)");
    std::shared_ptr<IExprTreeNode<double, double>> node2 = uetsf.makeShared(
        "2*t"
    );
    for(double const &ti : t){
        double const absDiff = std::abs(node->eval(ti)-node2->eval(ti));
        if(absDiff > eps) return false;
    }

    // Test case 21: t+(1)
    node = uetsf.makeShared("t+(1)");
    y = std::vector<double>({
        -9.00000000, -2.14000000, -1.72000000, -0.10000000,  0.66000000,
        1.00000000,  1.34000000,  2.10000000,  3.72000000,  4.14000000,
        11.00000000
    });
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 22: (-1)+t
    node = uetsf.makeShared("(-1)+t");
    y = std::vector<double>({
        -11.00000000,  -4.14000000,  -3.72000000,  -2.10000000,
        -1.34000000,  -1.00000000,  -0.66000000,   0.10000000,
        1.72000000,   2.14000000,   9.00000000
    });
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 23: (-1+1)+t
    node = uetsf.makeShared("(-1+1)+t");
    if(!validate<double, double>(node, t, t)) return false;

    // Test case 24: (1-(-1))
    node = uetsf.makeShared("(1-(-1))");
    y = std::vector<double>({2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2});
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 25: (1-(1))
    node = uetsf.makeShared("(1-(1))");
    y = std::vector<double>({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    if(!validate<double, double>(node, t, y)) return false;

    // Test case 26: abs(t)
    // TODO Rethink: Implement

    // Test case 27: 1+atan2(3*t-1, 1)
    // TODO Rethink: Implement

    // Test case 28: 1+atan2(3*t-1, (t-1)*2)
    // TODO Rethink: Implement

    // Test case 29: t+(-(t-1)) = t+(-(-1+t))
    // TODO Rethink: Implement



    // On test passed
    return true;
}


}