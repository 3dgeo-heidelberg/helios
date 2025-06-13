target_include_directories(
  helios
  PUBLIC
  "."
)

target_sources(
  helios
  PUBLIC
  "Tests.cpp"
)

target_sources(
  helios_test
  PUBLIC
  "CatchTests.cpp"
  "CatchDiscreteTimeTest.cpp"
  "CatchFunctionalPlatformTest.cpp"
  "CatchVoxelParsingTest.cpp"
  "CatchHPCTest.cpp"
  "CatchEnergyModelsTest.cpp"
  "CatchAssetLoadingTest.cpp"
  "CatchExprTreeTest.cpp"
)