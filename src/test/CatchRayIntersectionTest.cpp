#include <catch2/catch_test_macros.hpp>

#include <DetailedVoxel.h>
#include <maths/RayUtils.h>
#include <UniformNoiseSource.h>

static bool checkIntersection(const std::vector<double>& it) {
    if(it.empty()) return false;
    if(it[0] < 0.0 && it[1] < 0.0) return false;
    return true;
}

TEST_CASE("RayIntersection: Intersection and handling") {
    // Create voxels
    DetailedVoxel dv1(
        6, 6, 6, 2,
        std::vector<int>({0, 49}),
        std::vector<double>({
            0, 89.7859174, 0.0222306, 0, 0.1843913,
            0.1244739, 0.0644808, 3.1595603, 1, 0,
            0
        })
    );
    dv1.part = std::make_shared<ScenePart>();
    dv1.part->onRayIntersectionMode = "TRANSMITTIVE";
    DetailedVoxel dv2(
        6, 6, -2, 2,
        std::vector<int>({0, 120}),
        std::vector<double>({
            0, 89.9096456, 0.061645, 0, 0.1772976,
            0.3734215, 0.124348, 14.9217589, 1, 0,
            0
        })
    );
    dv2.part = std::make_shared<ScenePart>();
    dv2.part->onRayIntersectionMode = "TRANSMITTIVE";

    // Create rays (o := originWaypoint , v := normalized director vector)
    glm::dvec3 o1(6, 6, 9);             // Ray 1 originWaypoint
    glm::dvec3 v1(0, 0, -1);            // Ray 1 direction
    glm::dvec3 o2(-6, 6, 6);            // Ray 2 originWaypoint
    glm::dvec3 v2(1, 0, 0);             // Ray 2 direction
    glm::dvec3 o3(-6, 4, -5.5);         // Ray 3 originWaypoint
    glm::dvec3 v3(0.968, 0.061, 0.242); // Ray 3 direction
    glm::dvec3 o4(2, 4, -10);           // Ray 4 originWaypoint
    glm::dvec3 v4(0.371, 0.019, 0.928); // Ray 4 direction
    glm::dvec3 o5(9, 4, -10);           // Ray 5 originWaypoint
    glm::dvec3 v5(0.707, 0.035, 0.707); // Ray 5 direction
    glm::dvec3 o6(6, 6, 9);             // Ray 6 originWaypoint
    glm::dvec3 v6(0, 0, 1);             // Ray 6 direction
    glm::dvec3 o7(7, 6, 3);             // Ray 7 originWaypoint
    glm::dvec3 v7(0.40824829, 0.81649658, 0.40824829); // Ray 7 direction

    // Validate simple intersections are as expected
    REQUIRE(checkIntersection(dv1.getRayIntersection(o1, v1)));      // r1 must hit dv1
    REQUIRE(checkIntersection(dv1.getRayIntersection(o2, v2)));      // r2 must hit dv1
    REQUIRE_FALSE(checkIntersection(dv1.getRayIntersection(o3, v3))); // r3 must NOT hit dv1
    REQUIRE(checkIntersection(dv1.getRayIntersection(o4, v4)));      // r4 must hit dv1
    REQUIRE_FALSE(checkIntersection(dv1.getRayIntersection(o5, v5))); // r5 must NOT hit dv1
    REQUIRE_FALSE(checkIntersection(dv1.getRayIntersection(o6, v6))); // r6 must NOT hit dv1
    REQUIRE(checkIntersection(dv1.getRayIntersection(o7, v7)));      // r7 must hit dv1
    REQUIRE(checkIntersection(dv2.getRayIntersection(o1, v1)));      // r1 must hit dv2
    REQUIRE_FALSE(checkIntersection(dv2.getRayIntersection(o2, v2))); // r2 must NOT hit dv2
    REQUIRE(checkIntersection(dv2.getRayIntersection(o3, v3)));      // r3 must hit dv2
    REQUIRE(checkIntersection(dv2.getRayIntersection(o4, v4)));      // r4 must hit dv2
    REQUIRE_FALSE(checkIntersection(dv2.getRayIntersection(o5, v5))); // r5 must NOT hit dv2
    REQUIRE_FALSE(checkIntersection(dv2.getRayIntersection(o6, v6))); // r6 must NOT hit dv2
    REQUIRE_FALSE(checkIntersection(dv2.getRayIntersection(o7, v7))); // r7 must NOT hit dv2

    // Prepare semitransparent voxel test
    UniformNoiseSource<double> uns("1", 0.0, 1.0);
    // UNS with seed "1" generates:
    // (0.932557, 0.128124, 0.999041, 0.236089, 0.396581)
    // (0.387911, 0.669746, 0.935539, 0.846311, 0.313274)
    glm::dvec3 so(0, 0, 0); // Sub ray originWaypoint

    // Semitransparent voxel test for ray1
    so = o1;
    std::vector<double> it = dv1.getRayIntersection(so, v1);
    glm::dvec3 iip(so+it[0]*v1); // Inside Intersection Point
    glm::dvec3 oip = RayUtils::obtainPointAfterTraversing(
        *dv1.getAABB(), so, v1, 0.0);
    double ints = 0.0;
    IntersectionHandlingResult ihr =
        dv1.onRayIntersection(uns, v1, iip, oip, ints);
    REQUIRE(ihr.canRayContinue()); // Ray must be able to continue

    so = oip + 0.00001 * v1;
    it = dv2.getRayIntersection(so, v1);
    iip = glm::dvec3(so+it[0]*v1);
    oip = RayUtils::obtainPointAfterTraversing(*dv2.getAABB(), so, v1, 0.0);
    ihr = dv2.onRayIntersection(uns, v1, iip, oip, ints);
    REQUIRE(ihr.canRayContinue()); // Ray must be able to continue

    // Semitransparent voxel test for ray2
    so = o2;
    it = dv1.getRayIntersection(so, v2);
    iip = glm::dvec3(so+it[0]*v2);
    oip = RayUtils::obtainPointAfterTraversing(*dv1.getAABB(), so, v2, 0.0);
    ihr = dv1.onRayIntersection(uns, v2, iip, oip, ints);
    REQUIRE(ihr.canRayContinue()); // Ray must be able to continue

    // Semitransparent voxel test for ray3
    so = o3;
    it = dv2.getRayIntersection(so, v3);
    iip = glm::dvec3(so+it[0]*v3);
    oip = RayUtils::obtainPointAfterTraversing(*dv2.getAABB(), so, v3, 0.0);
    ihr = dv2.onRayIntersection(uns, v3, iip, oip, ints);
    REQUIRE_FALSE(ihr.canRayContinue()); // Ray must NOT be able to continue

    // Semitransparent voxel test for ray4
    so = o4;
    it = dv2.getRayIntersection(so, v4);
    iip = glm::dvec3(so+it[0]*v4);
    oip = RayUtils::obtainPointAfterTraversing(*dv2.getAABB(), so, v4, 0.0);
    ihr = dv2.onRayIntersection(uns, v4, iip, oip, ints);
    REQUIRE_FALSE(ihr.canRayContinue()); // Ray must NOT be able to continue

    so = oip + 0.00001 * v4;
    it = dv1.getRayIntersection(so, v4);
    iip = glm::dvec3(so+it[0]*v4);
    oip = RayUtils::obtainPointAfterTraversing(*dv1.getAABB(), so, v4, 0.0);
    ihr = dv1.onRayIntersection(uns, v4, iip, oip, ints);
    REQUIRE(ihr.canRayContinue()); // Ray must be able to continue

    // Semitransparent voxel test for ray7
    so = o7;
    it = dv1.getRayIntersection(so, v7);
    iip = glm::dvec3(so+it[0]*v7);
    oip = RayUtils::obtainPointAfterTraversing(*dv1.getAABB(), so, v7, 0.0);
    ihr = dv1.onRayIntersection(uns, v7, iip, oip, ints);
    REQUIRE(ihr.canRayContinue()); // Ray must be able to continue
}