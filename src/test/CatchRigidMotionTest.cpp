#include <catch2/catch_test_macros.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"

#include <MathConstants.h>
#include <maths/rigidmotion/RigidMotionException.h>
#include <maths/rigidmotion/RigidMotionEngine.h>
#include <maths/rigidmotion/RigidMotionR2Factory.h>
#include <maths/rigidmotion/RigidMotionR3Factory.h>
#include <scene/primitives/Triangle.h>
#include <scene/dynamic/DynMotionEngine.h>
#include <scene/dynamic/DynMotion.h>
#include <scene/dynamic/DynMovingObject.h>

TEST_CASE("Rigid motion test") {
    constexpr double eps = 1e-5;

    SECTION("RigidMotion: Pure R2 Motions") {
        rigidmotion::RigidMotionEngine rme;
        rigidmotion::RigidMotionR2Factory rm2f;
        arma::mat R2X(2, 5);
        for(size_t i = 0 ; i < 10 ; ++i){
            R2X.at(i) = (((double)i)-2.5)*std::pow(-1.0, (double)i);
        }

        SECTION("Identity R2") {
            RigidMotion f = rm2f.makeIdentity();
            arma::mat Y = rme.apply(f, R2X);
            arma::mat Z = abs(Y-R2X);
            REQUIRE(!arma::any(arma::vectorise(Z) > eps));
            REQUIRE(RigidMotion::SuperType::R2_BASE == f.findSuperType());
            REQUIRE(f.hasFixedPoints());
            REQUIRE(RigidMotion::Type::IDENTITY_R2 == f.findType());
            REQUIRE(f.findInvariantDimensionality() == 2);
            size_t dim;
            arma::mat L = rme.computeFixedPoints(f, dim);
            arma::mat EL = eye(2, 2);
            REQUIRE(!arma::any(arma::vectorise(abs(L-EL)) > eps));
        }

        SECTION("Translation R2") {
            colvec shift({1.67, -2.27});
            RigidMotion f = rm2f.makeTranslation(shift);
            arma::mat Y = rme.apply(f, R2X);
            arma::mat EY = R2X.each_col() + shift;
            arma::mat Z = abs(Y-EY);
            REQUIRE(!arma::any(arma::vectorise(Z) > eps));
            REQUIRE(RigidMotion::SuperType::R2_BASE == f.findSuperType());
            REQUIRE(!f.hasFixedPoints());
            REQUIRE(RigidMotion::Type::TRANSLATION_R2 == f.findType());
            REQUIRE(f.findInvariantDimensionality() == 2);
            try {
                size_t dim;
                arma::mat L = rme.computeFixedPoints(f, dim);
                FAIL("Should have thrown RigidMotionException");
            } catch (rigidmotion::RigidMotionException&) {
                SUCCEED();
            }
        }

        SECTION("Reflection R2") {
            colvec axis({2.1, -3.9});
            colvec X({-3, 1});
            RigidMotion f = rm2f.makeReflection(axis);
            colvec Y = rme.apply(f, X);
            colvec EY({0.81651376, 3.05504587});
            REQUIRE(!arma::any(arma::vectorise(abs(Y-EY)) > eps));
            REQUIRE(RigidMotion::SuperType::R2_REFLECTION == f.findSuperType());
            REQUIRE(f.hasFixedPoints());
            REQUIRE(RigidMotion::Type::REFLECTION_R2 == f.findType());
            REQUIRE(f.findInvariantDimensionality() == 1);
            size_t dim;
            arma::mat L = rme.computeFixedPoints(f, dim);
            if(L(0,0) < 0.0) L = -L;
            arma::mat EL("0.47409982; -0.8804711");
            REQUIRE(!arma::any(arma::vectorise(abs(L-EL)) > eps));

            arma::mat Y2 = rme.apply(f, R2X);
            arma::mat EY2(2, 5);
            EY2.at(0, 0) = 0.12385321;        EY2.at(1, 0) = 2.91284404;
            EY2.at(0, 1) = 0.69266055;        EY2.at(1, 1) = 0.14220183;
            EY2.at(0, 2) = 1.26146789;        EY2.at(1, 2) = -2.62844037;
            EY2.at(0, 3) = 1.83027523;        EY2.at(1, 3) = -5.39908257;
            EY2.at(0, 4) = 2.39908257;        EY2.at(1, 4) = -8.16972477;
            arma::mat Z = abs(Y2-EY2);
            REQUIRE(!arma::any(arma::vectorise(Z) > eps));

            axis = {2.1, 0.9};
            f = rm2f.makeReflection(axis);
            arma::mat Y3 = rme.apply(f, R2X);
            arma::mat EY3(2, 5);
            EY3.at(0, 0) = -0.63793103;         EY3.at(1, 0) = -2.84482759;
            EY3.at(0, 1) = -0.70689655;         EY3.at(1, 1) = -0.01724138;
            EY3.at(0, 2) = -0.77586207;         EY3.at(1, 2) = 2.81034483;
            EY3.at(0, 3) = -0.84482759;         EY3.at(1, 3) = 5.63793103;
            EY3.at(0, 4) = -0.9137931;          EY3.at(1, 4) = 8.46551724;
            Z = abs(Y3-EY3);
            REQUIRE(!arma::any(arma::vectorise(Z) > eps));
        }

        SECTION("Glide Reflection R2") {
            colvec axis({2.1, 1.9});
            RigidMotion f = rm2f.makeGlideReflection(axis, 2);
            arma::mat Y = rme.apply(f, R2X);
            arma::mat EY(2, 5);
            EY.at(0, 0) = 2.7262137;        EY.at(1, 0) = -1.29533046;
            EY.at(0, 1) = 0.93569001;       EY.at(1, 1) = 0.89419573;
            EY.at(0, 2) = -0.85483368;      EY.at(1, 2) = 3.08372191;
            EY.at(0, 3) = -2.64535737;      EY.at(1, 3) = 5.27324809;
            EY.at(0, 4) = -4.43588106;      EY.at(1, 4) = 7.46277428;
            arma::mat Z = abs(Y-EY);
            REQUIRE(!arma::any(arma::vectorise(Z) > eps));
            REQUIRE(RigidMotion::SuperType::R2_REFLECTION == f.findSuperType());
            REQUIRE(!f.hasFixedPoints());
            REQUIRE(RigidMotion::Type::GLIDE_REFLECTION_R2 == f.findType());
            REQUIRE(f.findInvariantDimensionality() == 1);
            size_t dim;
            arma::mat L = rme.computeAssociatedInvariant(f, dim);
            if(L(0, 0) < 0) L = -L;
            arma::mat EL("0.74153578; 0.67091332");
            REQUIRE(!arma::any(arma::vectorise(abs(L-EL)) > eps));

            f = rm2f.makeTranslation(2*axis/norm(axis));
            RigidMotion g = rm2f.makeReflection(axis);
            RigidMotion h = rme.compose(f, g);
            arma::mat Y2 = rme.apply(h, R2X);
            Z = abs(Y-Y2);
            REQUIRE(!arma::any(arma::vectorise(Z) > eps));
        }

        SECTION("Rotation R2") {
            colvec center = R2X.col(2);
            RigidMotion f = rm2f.makeRotation(-M_PI/3, center);
            arma::mat Y = rme.apply(f, R2X);
            arma::mat EY(2, 5);
            EY.at(0, 0) = 2.96410162;       EY.at(1, 0) = 2.96410162;
            EY.at(0, 1) = 2.23205081;       EY.at(1, 1) = 0.23205081;
            EY.at(0, 2) = 1.5;              EY.at(1, 2) = -2.5;
            EY.at(0, 3) = 0.76794919;       EY.at(1, 3) = -5.23205081;
            EY.at(0, 4) = 0.03589838;       EY.at(1, 4) = -7.96410162;
            arma::mat Z = abs(Y-EY);
            REQUIRE(!arma::any(arma::vectorise(Z) > eps));
            REQUIRE(RigidMotion::SuperType::R2_ROTATION == f.findSuperType());
            REQUIRE(f.hasFixedPoints());
            REQUIRE(RigidMotion::Type::ROTATION_R2 == f.findType());
            REQUIRE(f.findInvariantDimensionality() == 0);
            size_t dim;
            arma::mat L = rme.computeFixedPoints(f, dim);
            arma::mat EL("1.5; -2.5");
            REQUIRE(!arma::any(arma::vectorise(abs(L-EL)) > eps));
        }
    }

    SECTION("RigidMotion: Pure R3 Motions") {
        RigidMotionEngine rme;
        RigidMotionR3Factory rm3f;
        arma::mat R3X(3, 5);
        for(size_t i = 0 ; i < 15 ; ++i){
            R3X.at(i) = (((double)i)-2.5)*std::pow(-1.0, (double)i);
        }

        SECTION("Identity R3") {
            RigidMotion f = rm3f.makeIdentity();
            arma::mat Y = rme.apply(f, R3X);
            arma::mat Z = abs(Y-R3X);
            REQUIRE(!arma::any(arma::vectorise(Z) > eps));
            REQUIRE(RigidMotion::SuperType::R3_BASE == f.findSuperType());
            REQUIRE(f.hasFixedPoints());
            REQUIRE(RigidMotion::Type::IDENTITY_R3 == f.findType());
            REQUIRE(f.findInvariantDimensionality() == 3);
            size_t dim;
            arma::mat L = rme.computeFixedPoints(f, dim);
            arma::mat EL(
                "1 0 0;"
                "0 1 0;"
                "0 0 1"
            );
            REQUIRE(!arma::any(arma::vectorise(L-EL) > eps));
        }

        SECTION("Translation R3") {
            colvec shift({1.1, -3.39, 0.24});
            RigidMotion f = rm3f.makeTranslation(shift);
            arma::mat Y = rme.apply(f, R3X);
            arma::mat EY = R3X.each_col() + shift;
            arma::mat Z = abs(Y-EY);
            REQUIRE(!arma::any(arma::vectorise(Z) > eps));
            REQUIRE(RigidMotion::SuperType::R3_BASE == f.findSuperType());
            REQUIRE(!f.hasFixedPoints());
            REQUIRE(RigidMotion::Type::TRANSLATION_R3 == f.findType());
            REQUIRE(f.findInvariantDimensionality() == 3);
        }

        SECTION("Reflection R3") {
            colvec ortho({1.0, 0.5, 0.3});
            RigidMotion f = rm3f.makeReflection(ortho);
            arma::mat Y = rme.apply(f, R3X);
            arma::mat EY(
                "0.3358209 0.24626866 -0.82835821 1.41044776 -1.99253731; "
                "2.91791045 1.87313433 -6.6641791 11.45522388 -16.24626866; "
                "0.35074627 -2.2761194 4.20149254 -6.12686567 8.05223881"
            );
            arma::mat Z = abs(Y-EY);
            REQUIRE(!arma::any(arma::vectorise(Z) > eps));
            Y = rme.apply(f, Y);
            Z = abs(Y-R3X);
            REQUIRE(!arma::any(arma::vectorise(Z) > eps));
            REQUIRE(RigidMotion::SuperType::R3_REFLECTION == f.findSuperType());
            REQUIRE(f.hasFixedPoints());
            REQUIRE(RigidMotion::Type::REFLECTION_R3 == f.findType());
            REQUIRE(f.findInvariantDimensionality() == 2);
            size_t dim;
            arma::mat L = rme.computeFixedPoints(f, dim);
            if(L(0,0) < 0) L = -L;
            arma::mat EL("0.86386843; 0.43193421; 0.25916053");
            REQUIRE(!arma::any(arma::vectorise(abs(L-EL)) > eps));
        }

        SECTION("Glide Reflection R3") {
            colvec ortho({1.0, 0.5, 0.3});
            colvec shift({-4.0/5.0, 1, 1});
            RigidMotion f = rm3f.makeGlideReflection(ortho, shift);
            arma::mat Y = rme.apply(f, R3X);
            arma::mat EY = (
                "-0.4641791   -0.55373134  -1.62835821   0.61044776  -2.79253731;"
                "3.91791045   2.87313433  -5.6641791   12.45522388 -15.24626866;"
                "1.35074627  -1.2761194    5.20149254  -5.12686567   9.05223881"
            );
            arma::mat Z = abs(Y-EY);
            REQUIRE(!arma::any(arma::vectorise(Z) > eps));
            REQUIRE(RigidMotion::SuperType::R3_REFLECTION == f.findSuperType());
            REQUIRE(!f.hasFixedPoints());
            REQUIRE(RigidMotion::Type::GLIDE_REFLECTION_R3 == f.findType());
            REQUIRE(f.findInvariantDimensionality() == 2);
            size_t dim;
            arma::mat L = rme.computeAssociatedInvariant(f, dim);
            if(L(0, 0) < 0) L = -L;
            arma::mat EL("0.86386843; 0.43193421; 0.25916053");
            REQUIRE(!arma::any(arma::vectorise(abs(L-EL)) > eps));
            f = rm3f.makeGlideReflection(ortho, -shift);
            Y = rme.apply(f, Y);
            Z = abs(Y-R3X);
            REQUIRE(!arma::any(arma::vectorise(Z) > eps));
            try {
                f = rm3f.makeGlideReflection(ortho, colvec("1 1 1"));
                FAIL("Should have thrown RigidMotionException");
            } catch (rigidmotion::RigidMotionException&) {
                SUCCEED();
            }
        }

        SECTION("Rotation R3") {
            colvec axis({0.4, 0.2, 1.0});
            double theta = 4.5;
            RigidMotion f = rm3f.makeRotation(axis, theta);
            arma::mat Y = rme.apply(f, R3X);
            arma::mat EY(
                "1.4704453    0.92147891  -3.31340313   5.70532734  -8.09725156;"
                "1.49407244  -1.24669143   0.99931042  -0.75192941   0.5045484;"
                "-2.08699261  -2.51925328   7.12549917 -11.73174506  16.33799094"
            );
            arma::mat Z = abs(Y-EY);
            REQUIRE(!arma::any(arma::vectorise(Z) > eps));
            REQUIRE(RigidMotion::SuperType::R3_ROTATION == f.findSuperType());
            REQUIRE(f.hasFixedPoints());
            REQUIRE(RigidMotion::Type::ROTATION_R3 == f.findType());
            REQUIRE(f.findInvariantDimensionality() == 1);
            size_t dim;
            arma::mat L = rme.computeFixedPoints(f, dim);
            if(L(0, 0) < 0) L = -L;
            arma::mat EL("0.36514837; 0.18257419; 0.91287093");
            REQUIRE(!arma::any(arma::vectorise(abs(L-EL)) > eps));
            f = rm3f.makeRotation(axis, -theta);
            Y = rme.apply(f, Y);
            Z = abs(Y-R3X);
            REQUIRE(!arma::any(arma::vectorise(Z) > eps));
        }
    }

    SECTION("RigidMotion: Helios Dynamic Motions") {
        RigidMotionEngine rme;
        RigidMotionR3Factory rm3f;
        DynMotionEngine dme;

        // Build dynamic moving object for the tests
        std::vector<Primitive *> primitives;
        Vertex v0(-1.0, -1.0, 0.0);
        Vertex v1(1.0, -1.0, 0.0);
        Vertex v2(0.0, 1.0, 0.0);
        Vertex v3(0.0, 0.0, 1.0);
        Triangle tr0(v0, v1, v2);
        Triangle tr1(v0, v1, v3);
        Triangle tr2(v0, v2, v3);
        Triangle tr3(v1, v2, v3);
        primitives.push_back(&tr0);
        primitives.push_back(&tr1);
        primitives.push_back(&tr2);
        primitives.push_back(&tr3);
        DynMovingObject dmo("HRMTestDMO", primitives);
        dmo.computeCentroid();

        auto validate_dyn_motion = [&](const DynMotion& dm, const RigidMotion& rm, DynMovingObject dynObj) {
            arma::mat X = dynObj.positionMatrixFromPrimitives();
            arma::mat dmeY = dme.apply(dm, X, dynObj);
            arma::mat rmeY = rme.apply(rm, X);
            size_t m = dmeY.n_elem, n = rmeY.n_elem;
            REQUIRE(m == n);
            for(size_t i = 0 ; i < m ; ++i)
                REQUIRE(std::fabs(dmeY[i]-rmeY[i]) <= eps);
        };

        SECTION("Identity R3") {
            DynMovingObject dynObj = dmo;
            RigidMotion id = rm3f.makeIdentity();
            DynMotion dm(id, false);
            validate_dyn_motion(dm, id, dynObj);
        }

        SECTION("Translation R3") {
            DynMovingObject dynObj = dmo;
            RigidMotion tr = rm3f.makeTranslation(arma::colvec("-0.1;0.1;0"));
            DynMotion dm(tr, false);
            validate_dyn_motion(dm, tr, dynObj);
        }

        SECTION("Reflection R3") {
            DynMovingObject dynObj = dmo;
            arma::colvec O = dmo.getCentroid();
            RigidMotion rf = rme.compose(
                rm3f.makeTranslation(O),
                rme.compose(rm3f.makeReflectionZ(), rm3f.makeTranslation(-O))
            );
            DynMotion dm(rm3f.makeReflectionZ(), true);
            validate_dyn_motion(dm, rf, dynObj);
        }

        SECTION("Glide Reflection R3") {
            DynMovingObject dynObj = dmo;
            arma::colvec O = dmo.getCentroid();
            RigidMotion rf = rme.compose(
                rm3f.makeTranslation(O),
                rme.compose(
                    rm3f.makeGlideReflection(
                        arma::colvec("0;0;1"),
                        arma::colvec("0.5;0.5;0")
                    ),
                    rm3f.makeTranslation(-O)
                )
            );
            DynMotion dm(
                rm3f.makeGlideReflection(
                    arma::colvec("0;0;1"),
                    arma::colvec("0.5;0.5;0")
                ),
                true
            );
            validate_dyn_motion(dm, rf, dynObj);
        }

        SECTION("Rotation R3") {
            DynMovingObject dynObj = dmo;
            arma::colvec O = dmo.getCentroid();
            RigidMotion rot = rme.compose(
                rm3f.makeTranslation(O),
                rme.compose(rm3f.makeRotationZ(0.67), rm3f.makeTranslation(-O))
            );
            DynMotion dm(rm3f.makeRotationZ(0.67), true);
            validate_dyn_motion(dm, rot, dynObj);
        }

        SECTION("Helical R3") {
            DynMovingObject dynObj = dmo;
            arma::colvec O = dmo.getCentroid();
            RigidMotion hm = rme.compose(
                rm3f.makeTranslation(O),
                rme.compose(rm3f.makeHelicalZ(0.5, 1.0), rm3f.makeTranslation(-O))
            );
            DynMotion dm(rm3f.makeHelicalZ(0.5, 1.0), true);
            validate_dyn_motion(dm, hm, dynObj);
        }

        SECTION("Rotational Symmetry R3") {
            DynMovingObject dynObj = dmo;
            arma::colvec O = dmo.getCentroid();
            RigidMotion rs = rme.compose(
                rm3f.makeTranslation(O),
                rme.compose(
                    rm3f.makeRotationalSymmetryZ(0.34),
                    rm3f.makeTranslation(-O)
                )
            );
            DynMotion dm(rm3f.makeRotationalSymmetryZ(0.34), true);
            validate_dyn_motion(dm, rs, dynObj);
        }
    }
}

