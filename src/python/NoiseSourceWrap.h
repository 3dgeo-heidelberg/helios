
template<typename RealType>
class NoiseSourceWrap : public NoiseSource<RealType>
{
public:
  using NoiseSource<RealType>::NoiseSource;

  RealType noiseFunction() override
  {
    throw std::runtime_error("Called pure virtual function noiseFunction()");
  }
};

template<typename RealType>
class RandomNoiseSourceWrap : public RandomNoiseSource<RealType>
{
public:
  using RandomNoiseSource<RealType>::RandomNoiseSource;

  // Override the pure virtual method in Python
  std::string getRandomNoiseType() override
  {
    PYBIND11_OVERRIDE_PURE(std::string,                 // Return type
                           RandomNoiseSource<RealType>, // Parent class
                           getRandomNoiseType           // Function name
    );
  }
  RealType noiseFunction() override
  {
    PYBIND11_OVERLOAD_PURE(RealType,                    // Return type
                           RandomNoiseSource<RealType>, // Parent class
                           noiseFunction,               // Method name in C++
    );
  }
};
