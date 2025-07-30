#include <memory>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <scanner/detector/AbstractDetector.h>

// Trampoline class
class AbstractDetectorWrap : public AbstractDetector
{
public:
  using AbstractDetector::AbstractDetector; // Inherit constructors

  // Override the pure virtual function clone()
  std::shared_ptr<AbstractDetector> clone() override
  {
    PYBIND11_OVERRIDE_PURE(std::shared_ptr<AbstractDetector>, // Return type
                           AbstractDetector,                  // Parent class
                           clone,                             // Function name
    );
  }

  // Override any other virtual functions if necessary
  void _clone(std::shared_ptr<AbstractDetector> ad) override
  {
    PYBIND11_OVERRIDE(void,             // Return type
                      AbstractDetector, // Parent class
                      _clone,           // Function name
                      ad                // Arguments
    );
  }
};
