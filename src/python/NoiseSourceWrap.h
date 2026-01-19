
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
