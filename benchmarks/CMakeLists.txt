# Individual benchmark programs
add_executable(macro_playback_b macro_playback_b.cpp)
target_link_libraries(macro_playback_b PUBLIC benchmark::benchmark helios)
add_test(NAME macro_playback_b COMMAND macro_playback_b)

add_executable(empty_b empty_b.cpp)
target_link_libraries(empty_b PUBLIC benchmark::benchmark helios)
add_test(NAME empty_b COMMAND empty_b)

add_executable(scanning_device_sim_step_b scanning_device_sim_step_b.cpp)
target_link_libraries(scanning_device_sim_step_b PUBLIC benchmark::benchmark helios)
add_test(NAME scanning_device_sim_step_b COMMAND scanning_device_sim_step_b)
