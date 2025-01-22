#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

class PrimitiveWrap : public Primitive {
public:
    using Primitive::Primitive; // Inherit constructors

     Primitive* clone() override {
        PYBIND11_OVERRIDE_PURE(
            Primitive*,   // Return type
            Primitive,    // Parent class
            clone,        // Name of the function in C++
        );
    }
    // Override getAABB method
    AABB* getAABB() override {
        PYBIND11_OVERRIDE_PURE(
            AABB*,         // Return type
            Primitive,     // Parent class
            getAABB,       // Name of the function in C++
        );
    }

    // Override getCentroid method
    glm::dvec3 getCentroid() override {
        PYBIND11_OVERRIDE_PURE(
            glm::dvec3,    // Return type
            Primitive,     // Parent class
            getCentroid,   // Name of the function in C++
        );
    }

    // Override getIncidenceAngle_rad method
    double getIncidenceAngle_rad(const glm::dvec3& p, const glm::dvec3& d, const glm::dvec3& n) override {
        PYBIND11_OVERRIDE_PURE(
            double,        // Return type
            Primitive,     // Parent class
            getIncidenceAngle_rad,   // Name of the function in C++
            p, d, n        // Arguments
        );
    }

    // Override getRayIntersection method
    std::vector<double> getRayIntersection(const glm::dvec3& p, const glm::dvec3& d) override {
        PYBIND11_OVERRIDE_PURE(
            std::vector<double>,    // Return type
            Primitive,              // Parent class
            getRayIntersection,     // Name of the function in C++
            p, d                    // Arguments
        );
    }

    // Override getRayIntersectionDistance method
    double getRayIntersectionDistance(const glm::dvec3& p, const glm::dvec3& d) override {
        PYBIND11_OVERRIDE_PURE(
            double,        // Return type
            Primitive,     // Parent class
            getRayIntersectionDistance,   // Name of the function in C++
            p, d           // Arguments
        );
    }

    // Override getVertices method
    Vertex* getVertices() override {
        PYBIND11_OVERRIDE_PURE(
            Vertex*,       // Return type
            Primitive,     // Parent class
            getVertices    // Name of the function in C++
        );
    }

    // Override update method
    void update() override {
        PYBIND11_OVERRIDE_PURE(
            void,          // Return type
            Primitive,     // Parent class
            update         // Name of the function in C++
        );
    }
};
