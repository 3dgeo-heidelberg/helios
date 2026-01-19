#include <pybind11/pybind11.h>

#include <scanner/beamDeflector/AbstractBeamDeflector.h>

class AbstractBeamDeflectorWrap : public AbstractBeamDeflector {
public:
    using AbstractBeamDeflector::AbstractBeamDeflector;

    std::string getOpticsType() const override {
        PYBIND11_OVERRIDE_PURE(
            std::string,
            AbstractBeamDeflector,
            getOpticsType,
            );
    }

};