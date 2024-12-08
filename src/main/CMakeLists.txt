target_include_directories(
  helios
  PUBLIC
  "."
)

target_sources(
  helios++
  PRIVATE
  "ArgumentsParser.cpp"
  "Main.cpp"
  "LidarSim.cpp"
)
