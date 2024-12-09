target_include_directories(
  helios
  PUBLIC
  "."
  "raycast"
)

target_sources(
  helios
  PRIVATE
  "raycast/GroveKDTreeRaycaster.cpp"
  "raycast/KDGroveRaycaster.cpp"
  "raycast/KDTreeRaycaster.cpp"
)
