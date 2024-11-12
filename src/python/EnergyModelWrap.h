#include <pybind11/pybind11.h>
#include <memory>
#include <vector>
#include "EnergyModel.h"  // Include your EnergyModel header
#include "ScanningDevice.h"  // Include your ScanningDevice header

namespace py = pybind11;

class EnergyModelWrap : public EnergyModel {
public:
    // Constructor for wrapping
    EnergyModelWrap(ScanningDevice const &sd) : EnergyModel(sd) {}

    // Implement all pure virtual methods
    double computeIntensity(double const incidenceAngle, double const targetRange, Material const &mat, int const subrayRadiusStep) override {
        PYBIND11_OVERLOAD_PURE(double, EnergyModel, computeIntensity, incidenceAngle, targetRange, mat, subrayRadiusStep);
    }

    double computeReceivedPower(ModelArg const &args) override {
        PYBIND11_OVERLOAD_PURE(double, EnergyModel, computeReceivedPower, args);
    }

    double computeEmittedPower(ModelArg const &args) override {
        PYBIND11_OVERLOAD_PURE(double, EnergyModel, computeEmittedPower, args);
    }

    double computeTargetArea(ModelArg const &args) override {
        PYBIND11_OVERLOAD_PURE(double, EnergyModel, computeTargetArea, args);
    }

    double computeCrossSection(ModelArg const &args) override {
        PYBIND11_OVERLOAD_PURE(double, EnergyModel, computeCrossSection, args);
    }
};