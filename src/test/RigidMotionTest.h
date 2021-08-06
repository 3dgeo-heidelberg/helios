#pragma once

#include "BaseTest.h"
#include <MathConstants.h>
#include <maths/rigidmotion/RigidMotionException.h>
#include <maths/rigidmotion/RigidMotionEngine.h>
#include <maths/rigidmotion/RigidMotionR2Factory.h>
#include <maths/rigidmotion/RigidMotionR3Factory.h>


#include <armadillo>

using namespace arma;
using namespace rigidmotion;

namespace HeliosTests{

/**
 * @author: Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Rigid motion test
 */
class RigidMotionTest : public BaseTest{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Decimal precision for validation purposes
     */
    double eps = 0.00001;
    /**
     * @brief The rigid motion engine to be used by tests
     */
    RigidMotionEngine rme;
    /**
     * @brief The R2 rigid motion factory to be used by tests
     */
    RigidMotionR2Factory rm2f;
    /**
     * @brief The R3 rigid motion factory to be used by tests
     */
    RigidMotionR3Factory rm3f;
    /**
     * @brief Matrix of points in R2 to be used by tests
     */
    mat R2X;
    /**
     * @brief Matrix of points in R3 to be used by tests
     */
    mat R3X;

    // ***  CONSTRUCTOR  *** //
    // ********************* //
    /**
     * @brief Rigid motion test constructor
     */
    RigidMotionTest() : BaseTest("Rigid motion test"){}

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseTest::run
     */
    bool run() override;

    // ***  SUB-TESTS  *** //
    // ******************* //
    /**
     * @brief Test rigid motion implementation on its own
     * @return True if passed, false otherwise
     */
    bool testPureRigidMotion();
    /**
     * @brief Test rigid motion implementation adapted to Helios components
     * @return True if passed, false otherwise
     */
    bool testHeliosRigidMotion();

    // ***  PURE TESTS  *** //
    // ******************** //
    /**
     * @brief Test identity rigid motion in \f$\mathbb{R}^{2}\f$
     * @return True if passed, false otherwise
     */
    bool testPureIdentityR2();
    /**
     * @brief Test translation rigid motion in \f$\mathbb{R}^{2}\f$
     * @return True if passed, false otherwise
     */
    bool testPureTranslationR2();
    /**
     * @brief Test reflection rigid motion in \f$\mathbb{R}^{2}\f$
     * @return True if passed, false otherwise
     */
    bool testPureReflectionR2();
    /**
     * @brief Test glide reflection rigid motion in \f$\mathbb{R}^{2}\f$
     * @return True if passed, false otherwise
     */
    bool testPureGlideReflectionR2();
    /**
     * @brief Test rotation rigid motion in \f$\mathbb{R}^{2}\f$
     * @return True if passed, false otherwise
     */
    bool testPureRotationR2();
    /**
     * @brief Test identity rigid motion in \f$\mathbb{R}^{3}\f$
     * @return True if passed, false otherwise
     */
    bool testPureIdentityR3();
    /**
     * @brief Test translation rigid motion in \f$\mathbb{R}^{3}\f$
     * @return True if passed, false otherwise
     */
    bool testPureTranslationR3();
    /**
     * @brief Test reflection rigid motion in \f$\mathbb{R}^{3}\f$
     * @return True if passed, false otherwise
     */
    bool testPureReflectionR3();
    /**
     * @brief Test glide reflection rigid motion in \f$\mathbb{R}^{3}\f$
     * @return True if passed, false otherwise
     */
    bool testPureGlideReflectionR3();
    /**
     * @brief Test rotation rigid motion in \f$\mathbb{R}^{3}\f$
     * @return True if passed, false otherwise
     */
    bool testPureRotationR3();
    /**
     * @brief Test helical rigid motion in \f$\mathbb{R}^{3}\f$
     * @return True if passed, false otherwise
     */
    bool testPureHelicalR3();
    /**
     * @brief Test rotational symmetry rigid motion in \f$\mathbb{R}^{3}\f$
     * @return True if passed, false otherwise
     */
    bool testPureRotationalSymmetryR3();

    // ***  HELIOS TESTS  *** //
    // ********************** //
};

// ***  R U N  *** //
// *************** //
bool RigidMotionTest::run(){
    // Initialize data
    R2X = mat(2, 5);
    for(size_t i = 0 ; i < 10 ; ++i){
        R2X.at(i) = (((double)i)-2.5)*std::pow(-1.0, (double)i);
    }
    R3X = mat(3, 5);
    for(size_t i = 0 ; i < 15 ; ++i){
        R3X.at(i) = (((double)i)-2.5)*std::pow(-1.0, (double)i);
    }

    // Run tests
    if(!testPureRigidMotion()) return false;
    if(!testHeliosRigidMotion()) return false;
    return true;
}

// ***  SUB-TESTS  *** //
// ******************* //
bool RigidMotionTest::testPureRigidMotion(){
    // R2 tests
    if(!testPureIdentityR2()) return false;
    if(!testPureTranslationR2()) return false;
    if(!testPureReflectionR2()) return false;
    if(!testPureGlideReflectionR2()) return false;
    if(!testPureRotationR2()) return false;

    // R3 tests
    if(!testPureIdentityR3()) return false;
    if(!testPureTranslationR3()) return false;
    if(!testPureReflectionR3()) return false;
    if(!testPureGlideReflectionR3()) return false;
    if(!testPureRotationR3()) return false;
    if(!testPureHelicalR3()) return false;
    if(!testPureRotationalSymmetryR3()) return false;

    return true;
}
bool RigidMotionTest::testHeliosRigidMotion(){
    // TODO Rethink : To be implemented
    return true;
}

// ***  PURE TESTS  *** //
// ******************** //
bool RigidMotionTest::testPureIdentityR2(){
    RigidMotion f = rm2f.makeIdentity();
    mat Y = rme.apply(f, R2X);
    mat Z = abs(Y-R2X);
    bool passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    passed = RigidMotion::SuperType::R2_BASE == f.findSuperType();
    if(!passed) return passed;
    passed = f.hasFixedPoints();
    if(!passed) return passed;
    passed = RigidMotion::Type::IDENTITY_R2 == f.findType();
    if(!passed) return passed;
    passed = f.findInvariantDimensionality() == 2;
    if(!passed) return passed;
    size_t dim;
    mat L = rme.computeFixedPoints(f, dim);
    mat EL = eye(2, 2);
    passed = !any(vectorise(abs(L-EL)) > eps);
    if(!passed) return passed;

    return passed;
}
bool RigidMotionTest::testPureTranslationR2(){
    colvec shift(std::vector<double>({1.67, -2.27}));
    RigidMotion f = rm2f.makeTranslation(shift);
    mat Y = rme.apply(f, R2X);
    mat EY = R2X.each_col() + shift;
    mat Z = abs(Y-EY);
    bool passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    passed = RigidMotion::SuperType::R2_BASE == f.findSuperType();
    if(!passed) return passed;
    passed = !f.hasFixedPoints();
    if(!passed) return passed;
    passed = RigidMotion::Type::TRANSLATION_R2 == f.findType();
    if(!passed) return passed;
    passed = f.findInvariantDimensionality() == 2;
    if(!passed) return passed;
    try{
        passed = false;
        size_t dim;
        mat L = rme.computeFixedPoints(f, dim);
    }
    catch(RigidMotionException &ex){
        passed = true;
    }
    if(!passed) return passed;

    return passed;
}
bool RigidMotionTest::testPureReflectionR2(){
    colvec axis(std::vector<double>({2.1, -3.9}));
    colvec X(std::vector<double>({-3, 1}));
    RigidMotion f = rm2f.makeReflection(axis);
    colvec Y = rme.apply(f, X);
    colvec EY(std::vector<double>({0.81651376, 3.05504587}));
    bool passed = !any(vectorise(abs(Y-EY)) > eps);
    if(!passed) return passed;

    passed = RigidMotion::SuperType::R2_REFLECTION == f.findSuperType();
    if(!passed) return passed;
    passed = f.hasFixedPoints();
    if(!passed) return passed;
    passed = RigidMotion::Type::REFLECTION_R2 == f.findType();
    if(!passed) return passed;
    passed = f.findInvariantDimensionality() == 1;
    if(!passed) return passed;
    size_t dim;
    mat L = rme.computeFixedPoints(f, dim);
    if(L(0,0) > 0.0) L = -L; // Solve sign ambiguity wrt expected solution
    mat EL("-0.88047109992; -0.47409982304");
    passed = !any(vectorise(abs(L-EL)) > eps);
    if(!passed) return passed;

    mat Y2 = rme.apply(f, R2X);
    mat EY2(2, 5);
    EY2.at(0, 0) = 0.12385321;        EY2.at(1, 0) = 2.91284404;
    EY2.at(0, 1) = 0.69266055;        EY2.at(1, 1) = 0.14220183;
    EY2.at(0, 2) = 1.26146789;        EY2.at(1, 2) = -2.62844037;
    EY2.at(0, 3) = 1.83027523;        EY2.at(1, 3) = -5.39908257;
    EY2.at(0, 4) = 2.39908257;        EY2.at(1, 4) = -8.16972477;
    mat Z = abs(Y2-EY2);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    axis = std::vector<double>({2.1, 0.9});
    f = rm2f.makeReflection(axis);
    mat Y3 = rme.apply(f, R2X);
    mat EY3(2, 5);
    EY3.at(0, 0) = -0.63793103;         EY3.at(1, 0) = -2.84482759;
    EY3.at(0, 1) = -0.70689655;         EY3.at(1, 1) = -0.01724138;
    EY3.at(0, 2) = -0.77586207;         EY3.at(1, 2) = 2.81034483;
    EY3.at(0, 3) = -0.84482759;         EY3.at(1, 3) = 5.63793103;
    EY3.at(0, 4) = -0.9137931;          EY3.at(1, 4) = 8.46551724;
    Z = abs(Y3-EY3);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    return passed;
}
bool RigidMotionTest::testPureGlideReflectionR2(){
    colvec axis(std::vector<double>({2.1, 1.9}));
    RigidMotion f = rm2f.makeGlideReflection(axis, 2);
    mat Y = rme.apply(f, R2X);
    mat EY(2, 5);
    EY.at(0, 0) = 2.7262137;        EY.at(1, 0) = -1.29533046;
    EY.at(0, 1) = 0.93569001;       EY.at(1, 1) = 0.89419573;
    EY.at(0, 2) = -0.85483368;      EY.at(1, 2) = 3.08372191;
    EY.at(0, 3) = -2.64535737;      EY.at(1, 3) = 5.27324809;
    EY.at(0, 4) = -4.43588106;      EY.at(1, 4) = 7.46277428;
    mat Z = abs(Y-EY);
    bool passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    passed = RigidMotion::SuperType::R2_REFLECTION == f.findSuperType();
    if(!passed) return passed;
    passed = !f.hasFixedPoints();
    if(!passed) return passed;
    passed = RigidMotion::Type::GLIDE_REFLECTION_R2 == f.findType();
    if(!passed) return passed;
    passed = f.findInvariantDimensionality() == 1;
    if(!passed) return passed;

    f = rm2f.makeTranslation(2*axis/norm(axis));
    RigidMotion g = rm2f.makeReflection(axis);
    RigidMotion h = rme.compose(f, g);
    mat Y2 = rme.apply(h, R2X);
    Z = abs(Y-Y2);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    return passed;
}

bool RigidMotionTest::testPureRotationR2(){
    colvec center = R2X.col(2);
    RigidMotion f = rm2f.makeRotation(-M_PI/3, center);
    mat Y = rme.apply(f, R2X);
    mat EY(2, 5);
    EY.at(0, 0) = 2.96410162;       EY.at(1, 0) = 2.96410162;
    EY.at(0, 1) = 2.23205081;       EY.at(1, 1) = 0.23205081;
    EY.at(0, 2) = 1.5;              EY.at(1, 2) = -2.5;
    EY.at(0, 3) = 0.76794919;       EY.at(1, 3) = -5.23205081;
    EY.at(0, 4) = 0.03589838;       EY.at(1, 4) = -7.96410162;
    mat Z = abs(Y-EY);
    bool passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    passed = RigidMotion::SuperType::R2_ROTATION == f.findSuperType();
    if(!passed) return passed;
    passed = f.hasFixedPoints();
    if(!passed) return passed;
    passed = RigidMotion::Type::ROTATION_R2 == f.findType();
    if(!passed) return passed;
    passed = f.findInvariantDimensionality() == 0;
    if(!passed) return passed;
    size_t dim;
    mat L = rme.computeFixedPoints(f, dim);
    mat EL("1.5; -2.5");
    passed = !any(vectorise(abs(L-EL)) > eps);
    if(!passed) return passed;

    return passed;
}

bool RigidMotionTest::testPureIdentityR3(){
    RigidMotion f = rm3f.makeIdentity();
    mat Y = rme.apply(f, R3X);
    mat Z = abs(Y-R3X);
    bool passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    passed = RigidMotion::SuperType::R3_BASE == f.findSuperType();
    if(!passed) return passed;
    passed = f.hasFixedPoints();
    if(!passed) return passed;
    passed = RigidMotion::Type::IDENTITY_R3 == f.findType();
    if(!passed) return passed;
    passed = f.findInvariantDimensionality() == 3;
    if(!passed) return passed;
    size_t dim;
    mat L = rme.computeFixedPoints(f, dim);
    mat EL(
        "1 0 0;"
        "0 1 0;"
        "0 0 1"
    );
    passed = !any(vectorise(L-EL) > eps);
    if(!passed) return passed;

    return passed;
}

bool RigidMotionTest::testPureTranslationR3(){
    colvec shift(std::vector<double>({1.1, -3.39, 0.24}));
    RigidMotion f = rm3f.makeTranslation(shift);
    mat Y = rme.apply(f, R3X);
    mat EY = R3X.each_col() + shift;
    mat Z = abs(Y-EY);
    bool passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    passed = RigidMotion::SuperType::R3_BASE == f.findSuperType();
    if(!passed) return passed;
    passed = !f.hasFixedPoints();
    if(!passed) return passed;
    passed = RigidMotion::Type::TRANSLATION_R3 == f.findType();
    if(!passed) return passed;
    passed = f.findInvariantDimensionality() == 3;
    if(!passed) return passed;

    return passed;
}

bool RigidMotionTest::testPureReflectionR3(){
    colvec ortho(std::vector<double>({1.0, 0.5, 0.3}));
    RigidMotion f = rm3f.makeReflection(ortho);
    mat Y = rme.apply(f, R3X);
    mat EY(
        "0.3358209 0.24626866 -0.82835821 1.41044776 -1.99253731; "
        "2.91791045 1.87313433 -6.6641791 11.45522388 -16.24626866; "
        "0.35074627 -2.2761194 4.20149254 -6.12686567 8.05223881"
    );
    mat Z = abs(Y-EY);
    bool passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;
    Y = rme.apply(f, Y);
    Z = abs(Y-R3X);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    passed = RigidMotion::SuperType::R3_REFLECTION == f.findSuperType();
    if(!passed) return passed;
    passed = f.hasFixedPoints();
    if(!passed) return passed;
    passed = RigidMotion::Type::REFLECTION_R3 == f.findType();
    if(!passed) return passed;
    passed = f.findInvariantDimensionality() == 2;
    if(!passed) return passed;
    size_t dim;
    mat L = rme.computeFixedPoints(f, dim);
    if(L(0,0) < 0) L = -L; // Solve sign ambiguity wrt expected solution
    mat EL("0.86386843; 0.43193421; 0.25916053");
    passed = !any(vectorise(abs(L-EL)) > eps);
    if(!passed) return passed;

    f = rm3f.makeReflectionX();
    Y = rme.apply(f, R3X);
    EY = mat(
        "2.5   0.5  -3.5   6.5  -9.5;"
        "1.5   1.5  -4.5   7.5 -10.5;"
        "-0.5  -2.5   5.5  -8.5  11.5"
    );
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;
    f = rm3f.makeReflection(colvec(std::vector<double>({1.0, 0.0, 0.0})));
    Y = rme.apply(f, R3X);
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    f = rm3f.makeReflectionY();
    Y = rme.apply(f, R3X);
    EY = mat(
        "-2.5 -0.5  3.5 -6.5  9.5;"
        "-1.5 -1.5  4.5 -7.5 10.5;"
        "-0.5 -2.5  5.5 -8.5 11.5"
    );
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;
    f = rm3f.makeReflection(colvec(std::vector<double>({0.0, 1.0, 0.0})));
    Y = rme.apply(f, R3X);
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    f = rm3f.makeReflectionY();
    Y = rme.apply(f, R3X);
    EY = mat(
        "-2.5 -0.5  3.5 -6.5  9.5;"
        "-1.5 -1.5  4.5 -7.5 10.5;"
        "-0.5 -2.5  5.5 -8.5 11.5"
    );
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;
    f = rm3f.makeReflection(colvec(std::vector<double>({0.0, 1.0, 0.0})));
    Y = rme.apply(f, R3X);
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    f = rm3f.makeReflectionZ();
    Y = rme.apply(f, R3X);
    EY = mat(
        "-2.5  -0.5   3.5  -6.5   9.5;"
        "1.5   1.5  -4.5   7.5 -10.5;"
        "0.5   2.5  -5.5   8.5 -11.5;"
    );
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;
    f = rm3f.makeReflection(colvec(std::vector<double>({0.0, 0.0, 1.0})));
    Y = rme.apply(f, R3X);
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    f = rm3f.makeReflectionX();
    RigidMotion g = rm3f.makeReflectionY();
    f = rme.compose(f, g);
    g = rm3f.makeReflectionZ();
    f = rme.compose(f, g);
    Y = rme.apply(f, R3X);
    EY = mat(
        "2.5   0.5  -3.5   6.5  -9.5;"
        "-1.5  -1.5 4.5   -7.5  10.5;"
        "0.5   2.5  -5.5   8.5 -11.5;"
    );
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    return passed;
}

bool RigidMotionTest::testPureGlideReflectionR3(){
    colvec ortho(std::vector<double>({1.0, 0.5, 0.3}));
    colvec shift(std::vector<double>({-4.0/5.0, 1, 1}));
    RigidMotion f = rm3f.makeGlideReflection(ortho, shift);
    mat Y = rme.apply(f, R3X);
    mat EY = (
        "-0.4641791   -0.55373134  -1.62835821   0.61044776  -2.79253731;"
        "3.91791045   2.87313433  -5.6641791   12.45522388 -15.24626866;"
        "1.35074627  -1.2761194    5.20149254  -5.12686567   9.05223881"
    );
    mat Z = abs(Y-EY);
    bool passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    passed = RigidMotion::SuperType::R3_REFLECTION == f.findSuperType();
    if(!passed) return passed;
    passed = !f.hasFixedPoints();
    if(!passed) return passed;
    passed = RigidMotion::Type::GLIDE_REFLECTION_R3 == f.findType();
    if(!passed) return passed;
    passed = f.findInvariantDimensionality() == 2;
    if(!passed) return passed;

    f = rm3f.makeGlideReflection(ortho, -shift);
    Y = rme.apply(f, Y);
    Z = abs(Y-R3X);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    try{
        passed = false;
        f = rm3f.makeGlideReflection(ortho, colvec("1 1 1"));
    }
    catch(RigidMotionException &rmex){
        passed = true;
    }

    return passed;
}

bool RigidMotionTest::testPureRotationR3(){
    colvec axis(std::vector<double>({0.4, 0.2, 1.0}));
    double theta = 4.5;
    RigidMotion f = rm3f.makeRotation(axis, theta);
    mat Y = rme.apply(f, R3X);
    mat EY(
        "1.4704453    0.92147891  -3.31340313   5.70532734  -8.09725156;"
        "1.49407244  -1.24669143   0.99931042  -0.75192941   0.5045484;"
        "-2.08699261  -2.51925328   7.12549917 -11.73174506  16.33799094"
    );
    mat Z = abs(Y-EY);
    bool passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    passed = RigidMotion::SuperType::R3_ROTATION == f.findSuperType();
    if(!passed) return passed;
    passed = f.hasFixedPoints();
    if(!passed) return passed;
    passed = RigidMotion::Type::ROTATION_R3 == f.findType();
    if(!passed) return passed;
    passed = f.findInvariantDimensionality() == 1;
    if(!passed) return passed;
    size_t dim;
    mat L = rme.computeFixedPoints(f, dim);
    if(L(0, 0) > 0) L = -L; // Solve sign ambiguity wrt expected solution
    mat EL("-0.72434672620; -0.56025593648; 0.40178987778");
    passed = !any(vectorise(abs(L-EL)) > eps);
    if(!passed) return passed;

    f = rm3f.makeRotation(axis, -theta);
    Y = rme.apply(f, Y);
    Z = abs(Y-R3X);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    f = rm3f.makeRotationX(theta);
    Y = rme.apply(f, R3X);
    EY = mat(
        "-2.5        -0.5         3.5        -6.5         9.5;"
        "-0.80495876 -2.76001899  6.32499674 -9.8899745  13.45495225;"
        "-1.36089728 -0.93930568  3.23950863 -5.53971159  7.83991454"
    );
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    f = rm3f.makeRotationY(theta);
    Y = rme.apply(f, R3X);
    EY = mat(
        "1.01575456   2.54922319  -6.11420095   9.6791787  -13.24415645;"
        "1.5          1.5         -4.5          7.5        -10.5;"
        "-2.33842739   0.03822444   2.26197851  -4.56218147   6.86238442"
    );
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    f = rm3f.makeRotationZ(theta);
    Y = rme.apply(f, R3X);
    EY = mat(
        "1.99328468   1.57169308  -5.13667083   8.70164858 -12.26662633;"
        "2.1276316    0.17257136  -2.47277431   4.77297727  -7.07318022;"
        "-0.5         -2.5          5.5         -8.5         11.5"
    );
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    return passed;
}

bool RigidMotionTest::testPureHelicalR3(){
    colvec axis(std::vector<double>({0.4, 0.2, 1.0}));
    double theta = 4.5;
    double glide = 5.0;
    RigidMotion f = rm3f.makeHelical(axis, theta, glide);
    mat Y = rme.apply(f, R3X);
    mat EY(
        "3.29618716  2.74722077 -1.48766127  7.5310692  -6.2715097;"
        "2.40694337 -0.3338205   1.91218135  0.16094152  1.41741933;"
        "2.47736204  2.04510137 11.68985381 -7.16739041 20.90234559"
        );
    mat Z = abs(Y-EY);
    bool passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    passed = RigidMotion::SuperType::R3_ROTATION == f.findSuperType();
    if(!passed) return passed;
    passed = !f.hasFixedPoints();
    if(!passed) return passed;
    passed = RigidMotion::Type::HELICAL_R3 == f.findType();
    if(!passed) return passed;
    passed = f.findInvariantDimensionality() == 1;
    if(!passed) return passed;

    f = rm3f.makeHelical(axis, -theta, -glide);
    Y = rme.apply(f, Y);
    Z = abs(Y-R3X);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    f = rm3f.makeHelicalX(theta, glide);
    Y = rme.apply(f, R3X);
    EY = mat(
        "2.5         4.5         8.5        -1.5        14.5;"
        "-0.80495876 -2.76001899  6.32499674 -9.8899745  13.45495225;"
        "-1.36089728 -0.93930568  3.23950863 -5.53971159  7.83991454"
        );
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    f = rm3f.makeHelicalY(theta, glide);
    Y = rme.apply(f, R3X);
    EY = mat(
        "1.01575456   2.54922319  -6.11420095   9.6791787  -13.24415645;"
        "6.5          6.5          0.5         12.5         -5.5;"
        "-2.33842739   0.03822444   2.26197851  -4.56218147   6.86238442"
        );
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    f = rm3f.makeHelicalZ(theta, glide);
    Y = rme.apply(f, R3X);
    EY = mat(
        "1.99328468   1.57169308  -5.13667083   8.70164858 -12.26662633;"
        "2.1276316    0.17257136  -2.47277431   4.77297727  -7.07318022;"
        "4.5          2.5         10.5         -3.5         16.5"

        );
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    return passed;
}

bool RigidMotionTest::testPureRotationalSymmetryR3(){
    colvec axis(std::vector<double>({1.0, 0.5, 0.3}));
    double theta = 4.5;
    RigidMotion f = rm3f.makeRotationalSymmetry(axis, theta);
    mat Y = rme.apply(f, R3X);
    mat EY(
        "2.23713056   1.83545467  -5.90803989   9.98062512 -14.05321035;"
        "0.45443109  -2.15342901   3.85242692  -5.55142484   7.25042276;"
        "-1.8811537   -0.8624672    3.6060881   -6.349709     9.0933299"
    );
    mat Z = abs(Y-EY);
    bool passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;
    f = rm3f.makeRotationalSymmetry(axis, -theta);
    Y = rme.apply(f, Y);
    Z = abs(Y-R3X);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    passed = RigidMotion::SuperType::R3_ROTATIONAL_SYMMETRY==f.findSuperType();
    if(!passed) return passed;
    passed = f.hasFixedPoints();
    if(!passed) return passed;
    passed = RigidMotion::Type::ROTATIONAL_SYMMETRY_R3 == f.findType();
    if(!passed) return passed;
    passed = f.findInvariantDimensionality() == 0;
    if(!passed) return passed;
    size_t dim;
    mat L = rme.computeFixedPoints(f, dim);
    if(L(0, 0) > 0) L = -L; // Solve sign ambiguity wrt expected solution
    mat EL("0; 0; 0;");
    passed = !any(vectorise(abs(L-EL)) > eps);
    if(!passed) return passed;

    f = rm3f.makeRotationalSymmetryX(theta);
    Y = rme.apply(f, R3X);
    EY = mat(
        "2.5         0.5        -3.5         6.5        -9.5;"
        "-0.80495876 -2.76001899  6.32499674 -9.8899745  13.45495225;"
        "-1.36089728 -0.93930568  3.23950863 -5.53971159  7.83991454"
    );
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    f = rm3f.makeRotationalSymmetryY(theta);
    Y = rme.apply(f, R3X);
    EY = mat(
        "1.01575456   2.54922319  -6.11420095   9.6791787  -13.24415645;"
        "-1.5         -1.5          4.5         -7.5         10.5;"
        "-2.33842739   0.03822444   2.26197851  -4.56218147   6.86238442"
    );
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    f = rm3f.makeRotationalSymmetryZ(theta);
    Y = rme.apply(f, R3X);
    EY = mat(
        "1.99328468   1.57169308  -5.13667083   8.70164858 -12.26662633;"
        "2.1276316    0.17257136  -2.47277431   4.77297727  -7.07318022;"
        "0.5          2.5         -5.5          8.5        -11.5"
    );
    Z = abs(Y-EY);
    passed = !any(vectorise(Z) > eps);
    if(!passed) return passed;

    return passed;
}


}