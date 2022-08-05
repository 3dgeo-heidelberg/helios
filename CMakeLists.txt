# CMake project
cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(helios LANGUAGES CXX)
set(PROJECT_NAME "Helios++")

# Check last git hash
include(cmake/CMakeGitHash.cmake)
GitHashSetup()

# Configure CMake
message("Configuring CMake ...")
include(cmake/CMakeConfig.cmake)
message("CMake configured!\n\n")

# Handle sources
message("Handling sources ...")
include(cmake/CMakeSources.cmake)
message("Sources handled!\n\n")

# Load libraries
message("Loading libraries ...")
include(cmake/CMakeLibraries.cmake)
message("Libraries loaded!\n\n")

# Python bindings
message("Configuring python bindings ...")
include(cmake/CMakePybinds.cmake)
message("Python bindings configured!\n\n")

# Custom debug flags
message("Handling debug flags (if any) ...")
include(cmake/CMakeDebugFlags.cmake)
message("Debug flags handled!\n\n")

# Build executable
message("Building Helios++ ...")
include(cmake/CMakeBuildExec.cmake)
message("Helios++ built!\n\n")

# CMake finished
message("${PROJECT_NAME} CMake finished!")
