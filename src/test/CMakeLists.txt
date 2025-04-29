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
)

target_sources(
  helios_test
  PUBLIC
  "CatchDiscreteTimeTest.cpp"
)