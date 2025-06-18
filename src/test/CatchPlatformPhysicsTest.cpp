#include <catch2/catch_test_macros.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"

#include <HelicopterPlatform.h>
#include <MathConstants.h>

TEST_CASE("Platform physics test") {
    constexpr double eps = 0.0001;

    SECTION("PlatformPhysics: Roll only rotations") {
        HelicopterPlatform hp;
        double angle;
        double step = 0.09;
        double newRoll, pitch, yaw;
        for(size_t i=1 ; ; i++){
            angle = i*step;
            if(angle >= PI_2) break;
            hp.rotate(angle, 0.0, 0.0);
            hp.getRollPitchYaw(newRoll, pitch, yaw);
            if(newRoll < 0.0) newRoll += PI_2;
            double diff = newRoll - angle;
            REQUIRE(diff >= -eps);
            REQUIRE(diff <= eps);
        }
    }

    SECTION("PlatformPhysics: Pitch only rotations") {
        HelicopterPlatform hp;
        double angle;
        double step = 0.03;
        double roll, newPitch, yaw;
        for(size_t i=1 ; ; i++){
            angle = i*step - PI_HALF;
            if(angle >= PI_HALF) break;
            hp.rotate(0.0, angle, 0.0);
            hp.getRollPitchYaw(roll, newPitch, yaw);
            double diff = newPitch - angle;
            REQUIRE(diff >= -eps);
            REQUIRE(diff <= eps);
        }
    }

    SECTION("PlatformPhysics: Yaw only rotations") {
        HelicopterPlatform hp;
        double angle;
        double step = 0.09;
        double roll, pitch, newYaw;
        for(size_t i=1 ; ; i++){
            angle = i*step;
            if(angle >= PI_2) break;
            hp.rotate(0.0, 0.0, angle);
            hp.getRollPitchYaw(roll, pitch, newYaw);
            if(newYaw < 0.0) newYaw += PI_2;
            double diff = newYaw - angle;
            REQUIRE(diff >= -eps);
            REQUIRE(diff <= eps);
        }
    }

    SECTION("PlatformPhysics: Roll and pitch rotations") {
        HelicopterPlatform hp;
        double angle1, angle2;
        double step1 = 0.09, step2 = 0.03;
        double newRoll, newPitch, yaw;
        for(size_t i = 1 ; ; i++){
            angle1 = i*step1;
            angle2 = i*step2 - PI_HALF;
            if(angle1 >= PI_2) break;
            hp.rotate(angle1, angle2, 0.0);
            hp.getRollPitchYaw(newRoll, newPitch, yaw);
            if(newRoll < 0.0) newRoll += PI_2;
            double diff1 = newRoll - angle1;
            double diff2 = newPitch - angle2;
            REQUIRE(diff1 >= -eps);
            REQUIRE(diff1 <= eps);
            REQUIRE(diff2 >= -eps);
            REQUIRE(diff2 <= eps);
        }
    }

    SECTION("PlatformPhysics: Roll and yaw rotations") {
        HelicopterPlatform hp;
        double angle1, angle2;
        double step1 = 0.09, step2 = 0.09;
        double newRoll, pitch, newYaw;
        for(size_t i = 1 ; ; i++){
            angle1 = i*step1;
            angle2 = i*step2;
            if(angle1 >= PI_2) break;
            hp.rotate(angle1, 0.0, angle2);
            hp.getRollPitchYaw(newRoll, pitch, newYaw);
            if(newRoll < 0.0) newRoll += PI_2;
            if(newYaw < 0.0) newYaw += PI_2;
            double diff1 = newRoll - angle1;
            double diff2 = newYaw - angle2;
            REQUIRE(diff1 >= -eps);
            REQUIRE(diff1 <= eps);
            REQUIRE(diff2 >= -eps);
            REQUIRE(diff2 <= eps);
        }
    }

    SECTION("PlatformPhysics: Pitch and yaw rotations") {
        HelicopterPlatform hp;
        double angle1, angle2;
        double step1 = 0.03, step2 = 0.09;
        double roll, newPitch, newYaw;
        for(size_t i = 1 ; ; i++){
            angle1 = i*step1 - PI_HALF;
            angle2 = i*step2;
            if(angle2 >= PI_2) break;
            hp.rotate(0.0, angle1, angle2);
            hp.getRollPitchYaw(roll, newPitch, newYaw);
            if(newYaw < 0.0) newYaw += PI_2;
            double diff1 = newPitch - angle1;
            double diff2 = newYaw - angle2;
            REQUIRE(diff1 >= -eps);
            REQUIRE(diff1 <= eps);
            REQUIRE(diff2 >= -eps);
            REQUIRE(diff2 <= eps);
        }
    }

    SECTION("PlatformPhysics: Roll, pitch and yaw rotations") {
        HelicopterPlatform hp;
        double angle1, angle2, angle3;
        double step1 = 0.09, step2 = 0.03, step3 = -0.06;
        double newRoll, newPitch, newYaw;
        for(size_t i = 1 ; ; i++){
            angle1 = i*step1;
            angle2 = i*step2 - PI_HALF;
            angle3 = i*step3;
            if(angle1 >= PI_2) break;
            hp.rotate(angle1, angle2, angle3);
            hp.getRollPitchYaw(newRoll, newPitch, newYaw);
            if(newRoll < 0.0) newRoll += PI_2;
            if(newYaw < 0.0) newYaw += PI_2;
            double diff1 = newRoll - angle1;
            double diff2 = newPitch - angle2;
            double diff3 = newYaw - (angle3+PI_2);
            REQUIRE(diff1 >= -eps);
            REQUIRE(diff1 <= eps);
            REQUIRE(diff2 >= -eps);
            REQUIRE(diff2 <= eps);
            REQUIRE(diff3 >= -eps);
            REQUIRE(diff3 <= eps);
        }
    }
}


