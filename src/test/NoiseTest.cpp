#include <catch2/catch_test_macros.hpp>

#include <helios/noise/NormalNoiseSource.h>
#include <helios/noise/UniformNoiseSource.h>

TEST_CASE("Noise sources test")
{
  // Randomness generators
  RandomnessGenerator<double> rg1(2.0);
  RandomnessGenerator<double> rg2(3.0);

  // Noise sources
  UniformNoiseSource<double> uns1(rg1);
  UniformNoiseSource<double> uns2(rg2);
  UniformNoiseSource<double> uns3(rg1);
  UniformNoiseSource<double> uns4(rg1);
  UniformNoiseSource<double> uns5(rg2);
  NormalNoiseSource<double> nns1(rg1);
  NormalNoiseSource<double> nns2(rg2);
  NormalNoiseSource<double> nns3(rg1);
  NormalNoiseSource<double> nns4(rg1);

  // Configuration
  uns3.setFixedLifespan(3); // Fixed, renew each 3 uses
  uns4.setFixedLifespan(0); // Fixed, eternal
  uns4.fixedRenew();
  uns5.configureUniformNoise(-3, 3);

  double cache;

  // Noise tests
  REQUIRE(uns1.next() == uns3.next());
  REQUIRE(uns2.next() != uns3.next());
  REQUIRE(uns3.next() == uns4.next());
  REQUIRE(uns1.next() == uns3.next());
  REQUIRE(uns1.next() != uns3.next());
  REQUIRE(nns4.next() != uns4.next());
  REQUIRE(nns4.next() != nns4.next());
  REQUIRE(uns2.next() != uns2.next());

  cache = uns4.next();
  REQUIRE(uns4.next() == cache);
  uns4.fixedRenew();
  REQUIRE(uns4.next() != cache);
  REQUIRE(uns4.next() == uns4.next());

  nns1.configureNormalNoise(4.0, 2.0);
  nns3.configureNormalNoise(4.0, 2.0);
  REQUIRE(nns1.next() == nns3.next());
  nns1.next();
  REQUIRE(nns1.next() != nns3.next());

  for (size_t i = 0; i < 32; i++) {
    cache = uns5.next();
    REQUIRE(cache >= -3.0);
    REQUIRE(cache <= 3.0);
  }
  uns5.setClipEnabled(true).setClipMin(-1.0).setClipMax(1.0); // Enable clip
  for (size_t i = 0; i < 32; i++) {
    cache = uns5.next();
    REQUIRE(cache >= -1.0);
    REQUIRE(cache <= 1.0);
  }

  // Copy-move tests
  UniformNoiseSource<double> uns1c = uns1;
  REQUIRE(uns1.next() == uns1c.next());
  uns1c.next();
  REQUIRE(uns1.next() != uns1c.next());
  uns1.next();
  UniformNoiseSource<double> uns1m = std::move(uns1c);
  REQUIRE(uns1.next() == uns1m.next());
  uns1m.next();
  REQUIRE(uns1.next() != uns1m.next());
  NormalNoiseSource<double> nns1c(nns1);
  REQUIRE(nns1.next() == nns1c.next());
  nns1c.next();
  REQUIRE(nns1.next() != nns1c.next());
  nns1.next();
  NormalNoiseSource<double> nns1m(std::move(nns1c));
  REQUIRE(nns1.next() == nns1m.next());
  nns1m.next();
  REQUIRE(nns1.next() != nns1m.next());
}
