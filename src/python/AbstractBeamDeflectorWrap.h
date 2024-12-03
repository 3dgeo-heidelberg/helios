#include <pybind11/pybind11.h>

#include <scanner/beamDeflector/AbstractBeamDeflector.h>

class AbstractBeamDeflectorWrap : public AbstractBeamDeflector {
public:
    using AbstractBeamDeflector::AbstractBeamDeflector;

    std::shared_ptr<AbstractBeamDeflector> clone() override {
        PYBIND11_OVERRIDE_PURE(
            std::shared_ptr<AbstractBeamDeflector>,  // Return type
            AbstractBeamDeflector,                  // Parent class
            clone                                  // Function name
        );
    }

    void doSimStep() override {
        PYBIND11_OVERRIDE_PURE(
            void,                   // Return type
            AbstractBeamDeflector,  // Parent class
            doSimStep               // Function name
        );
    }

    std::string getOpticsType() const override {
        PYBIND11_OVERRIDE_PURE(
            std::string,            // Return type
            AbstractBeamDeflector,  // Parent class
            getOpticsType           // Function name
        );
    }
    

};