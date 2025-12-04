# Individual benchmark programs
add_executable(testing_b testing_b.cpp)
target_link_libraries(testing_b PUBLIC benchmark::benchmark helios)
add_test(NAME testing_b COMMAND testing_b)
