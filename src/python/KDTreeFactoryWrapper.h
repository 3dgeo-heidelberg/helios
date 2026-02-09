#include <adt/kdtree/KDTreeFactory.h>

class KDTreeFactoryWrap : public KDTreeFactory
{
public:
  using KDTreeFactory::KDTreeFactory; // Inherit constructors

  // Override pure virtual clone method
  KDTreeFactory* clone() const override
  {
    PYBIND11_OVERLOAD_PURE(KDTreeFactory*, // Return type
                           KDTreeFactory,  // Parent class
                           clone           // Function name
    );
  }

  KDTreeNodeRoot* makeFromPrimitivesUnsafe(
    std::vector<Primitive*>& primitives,
    bool const computeStats = false,
    bool const reportStats = false) override
  {
    PYBIND11_OVERLOAD_PURE(KDTreeNodeRoot*,          // Return type
                           KDTreeFactory,            // Parent class
                           makeFromPrimitivesUnsafe, // Method name
                           primitives,               // Arguments
                           computeStats,
                           reportStats);
  }
};
