# Initialize the CMake project
cmake_minimum_required(VERSION 3.18)

# If we are running in a Conda environment, we automatically
# add the Conda env prefix to the CMAKE_PREFIX_PATH
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  #TODO: Windows Conda environments are structured differently,
  #      how unfortunate is this?
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}/Library")
endif()

# Set the version strings for the project
if(SKBUILD)
  set(HELIOS_VERSION ${SKBUILD_PROJECT_VERSION})
  set(HELIOS_VERSION_FULL ${SKBUILD_PROJECT_VERSION_FULL})
else()
  set(HELIOS_VERSION "1.3.0")
  set(HELIOS_VERSION_FULL "1.3.0")
endif()

project(Helios++
  VERSION ${HELIOS_VERSION}
  LANGUAGES C CXX
  DESCRIPTION "Helios software for LiDAR simulations")

# We also store relevant CMake modules in the cmake/ directory
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

# Set CMake policies for this project

# We allow <Package>_ROOT (env) variables for locating dependencies
cmake_policy(SET CMP0074 NEW)
# We allow target_sources to convert relative paths to absolute paths
cmake_policy(SET CMP0076 NEW)

# Initialize some default paths
include(GNUInstallDirs)

# Define the minimum C++ standard that is required
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# For Python bindings, we need to enable position-independent code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Compilation options
option(BUILD_PYTHON "Enable building of Python bindings" ON)
option(BUILD_DOCS "Enable building of documentation" ON)
option(HELIOS_DATA_ANALYTICS "Enable data analytics mode used for debugging internal state" OFF)
option(HELIOS_BUDDING_METRICS "Enable budding metrics for debugging" OFF)
option(HELIOS_PCL "Enable PCL support for debugging visualization" OFF)

# Add a library for Helios that both the executable and the Python bindings can link against
add_library(helios)

# Find external dependencies
find_package(GDAL REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(glm REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread regex filesystem serialization iostreams random)
find_package(Armadillo REQUIRED)

# LasTools's CMake is not Modern CMake yet, so we have to use old-style find_package
# that uses the provoded FindLASTools.cmake file
find_package(LASTools REQUIRED)

# We only build against PCL if explicitly requested
if(HELIOS_PCL)
  find_package(PCL COMPONENTS common features io io_ply visualization)
  target_link_libraries(helios PUBLIC ${PCL_LIBRARIES})
endif()

# Add all the dependencies to the Helios library
target_link_libraries(helios
  PUBLIC
    GDAL::GDAL
    tinyxml2::tinyxml2
    glm::glm
    Boost::system
    Boost::thread
    Boost::regex
    Boost::filesystem
    Boost::serialization
    Boost::iostreams
    Boost::random
    ${LASTOOLS_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
)

target_include_directories(helios
  PUBLIC
    ${ARMADILLO_INCLUDE_DIRS}
    ${LASTOOLS_INCLUDE_DIRS}
)

# Expose version information to the C++ sources
target_compile_definitions(helios
  PUBLIC
    HELIOS_VERSION="${HELIOS_VERSION_FULL}"
    HELIOS_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)

# Ensure Windows-specific compile options
if(WIN32 OR MSVC)
  target_compile_definitions(helios PUBLIC WIN32_LEAN_AND_MEAN _OS_WINDOWS_)
endif()

# Enable data analytics code if requested
if(HELIOS_DATA_ANALYTICS)
  target_compile_definitions(helios PUBLIC DATA_ANALYTICS=${HELIOS_DATA_ANALYTICS})
endif()
if(HELIOS_BUDDING_METRICS)
  target_compile_definitions(helios PUBLIC BUDDING_METRICS=ON)
endif()

# Add the HelIOS++ executable
add_executable(helios++)

target_link_libraries(helios++ PUBLIC helios)

if(BUILD_PYTHON)
  find_package(Python COMPONENTS Interpreter Development)
  find_package(Boost REQUIRED COMPONENTS python)

  add_library(_pyhelios MODULE)

  target_link_libraries(_pyhelios
  PUBLIC
    helios
    Boost::python
    Python::Module
  )

  # Control the output name of the produced shared library
  set_target_properties(_pyhelios PROPERTIES PREFIX "")
  set_target_properties(_pyhelios PROPERTIES OUTPUT_NAME "_pyhelios")
  if(WIN32)
    set_target_properties(_pyhelios PROPERTIES SUFFIX ".pyd")
  endif()
endif()

# Traverse the source tree to add all relevant sources
add_subdirectory(src)

# Add documentation building through the CMake build system
if(BUILD_DOCS)
  add_subdirectory(doc)
endif()

# Add installation rules for the Helios++ executable and Python bindings library

# Define an installation rule that copies the executable to our Python package
install(
  TARGETS
    helios++
  DESTINATION pyhelios/bin
)

if(BUILD_PYTHON)
  install(
    TARGETS
      _pyhelios
    DESTINATION .
  )
endif()

include(FeatureSummary)
feature_summary(WHAT ALL)
