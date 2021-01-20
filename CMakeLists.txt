cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(helios LANGUAGES CXX)

# ---  C O N F I G U R A T I O N  --- #
# ----------------------------------- #
# Setup compiler flags
# Default is RELWITHDEBINFO
# Change on command line e.g. with "-DCMAKE_BUILD_TYPE=RELEASE"

# Configure flags
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RELEASE)
endif()
if(WIN32 OR MSVC) # Windows flags
  set(CMAKE_CXX_FLAGS_RELEASE "/MD /O2 /Ob2 /DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG "")
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_FLAGS "-D_OS_WINDOWS_  /EHsc")
else() # Linux flags
  #set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG -Wall")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -Wall")
  set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall")
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_FLAGS "-pthread -lz -Wno-deprecated")
endif()

# More configuration
if(NOT PYTHON_VERSION)
  set(PYTHON_VERSION 37)
endif()
string(TOUPPER ${CMAKE_BUILD_TYPE} BUILD_TYPE)

# Initial messages
if(WIN32 OR MSVC)
  message("CMake compiling for WINDOWS")
else()
  message("CMake compiling for LINUX")
endif()
message("CMake compiling from : '${CMAKE_CURRENT_SOURCE_DIR}'")
message("CMAKE BUILD TYPE : '${CMAKE_BUILD_TYPE}'")
message("BUILD TYPE : '${BUILD_TYPE}'")
if(PYTHON_BINDING)
  message("PYTHON VERSION : 'python${PYTHON_VERSION}'")
else()
  message("NO PYTHON BINDINGS!")
endif()
message("--------------------\n")




# ---  S O U R C E S  --- #
# ----------------------- #
# Define helios executable ###################
file(GLOB_RECURSE sources CONFIGURE_DEPENDS src/*.cpp src/*.hpp src/*.h)
include_directories(
    "src/"
    "src/assetloading/"
    "src/assetloading/geometryfilter/"
    "src/platform/"
    "src/scanner/"
    "src/scanner/beamDeflector/"
    "src/scanner/detector/"
    "src/scene/"
    "src/scene/primitives/"
    "src/surveyplayback/"
    "src/noise/"
    "src/maths/"
    "src/util/"
    "src/util/logger/"
    "src/test/"
)




# ---  P A C K A G E S  --- #
# ------------------------- #
# Python bindings
if(PYTHON_BINDING)
  if(NOT PYTHON_PATH)
    find_package(PythonLibs)
  else()
    set(PYTHON_INCLUDE_DIRS ${PYTHON_PATH}/include)
	set(PYTHON_LIBRARIES ${PYTHON_PATH}/libs/python${PYTHON_VERSION}.lib)
  endif()
  message("INCLUDING '${PYTHON_PATH}'")
  include_directories (${PYTHON_INCLUDE_DIRS})
  message("PYTHON_INCLUDE_DIRS: ${PYTHON_INCLUDE_DIRS}")
  message("PYTHON_LIBRARIES: ${PYTHON_LIBRARIES}")
  include_directories("src/pybinds/")
  file(GLOB_RECURSE pysources CONFIGURE_DEPENDS
      src/pybinds/*.cpp src/pybinds/*.hpp src/pybinds/*.h)
else()
  message("NO PYTHON BINDINGS!")
endif()

#GLM
set(GLM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/glm")
message("INCLUDING '${GLM_DIR}'")
include_directories(${GLM_DIR})

# GDAL
if(WIN32 OR MSVC)
  set(GDAL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/gdal/include")
  set(GDAL_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/lib/gdal/lib/gdal_i.lib")
endif()
find_package(GDAL 3.0 REQUIRED)
if(GDAL_FOUND)
  message(STATUS "gdal found")
  message("GDAL_INCLUDE_DIRS: " ${GDAL_INCLUDE_DIRS})
  message("GDAL_LIBRARIES: " ${GDAL_LIBRARIES})
  include_directories(${GDAL_INCLUDE_DIRS})
endif()

# ZLIB (Linux only)
if(UNIX)
  find_package(ZLIB)
  if(ZLIB_FOUND)
    message(STATUS "zlib found")
    message("ZLIB_LIBRARIES: " ${ZLIB_LIBRARIES})
    message("ZLIB_INCLUDE_DIRS: " ${ZLIB_INCLUDE_DIRS})
    include_directories(${ZLIB_INCLUDE_DIRS})
  endif()
endif()

#BOOST
if(${BUILD_TYPE} STREQUAL DEBUG)
  set(Boost_DETAILED_FAILURE_MSG ON)
  set(Boost_DEBUG 1)
else()
  set(Boost_DETAILED_FAILURE_MSG OFF)
  set(Boost_DEBUG 0)
endif()
set(Boost_ADDITIONAL_VERSIONS 1.71.0 1.71 1.72.0 1.72)
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_RELEASE_LIBS ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_NO_SYSTEM_PATHS ON)
set(Boost_NO_BOOST_CMAKE OFF)
if(WIN32 OR MSVC)  # Specific boost stuff for Windows
  set(Boost_THREADAPI win32)
  set(BOOST_INCLUDEDIR "lib/boost")
  set(BOOST_LIBRARYDIR "lib/boost/stage/lib")
  set(BASE_BOOST_COMPONENTS system thread regex filesystem serialization iostreams zlib)
else()  # Specific boost stuff for Linux
  #set(Boost_COMPILER "-vc")
  set(Boost_THREADAPI pthread)
  set(BOOST_ROOT "lib/boost")
  set(BASE_BOOST_COMPONENTS system thread regex filesystem serialization iostreams)
endif()

# LASlib
if(WIN32)
	set(LASlib ${CMAKE_SOURCE_DIR}/lib/LAStools/LASlib/lib/Release/LASlib.lib)
	if(EXISTS "${LASlib}")
	else()
	set(LASlib ${CMAKE_SOURCE_DIR}/lib/LAStools/LASlib/lib/LASlib.lib)
	endif()
else()
	set(LASlib ${CMAKE_SOURCE_DIR}/lib/LAStools/LASlib/lib/libLASlib.a)
endif()
include_directories(
    ${CMAKE_SOURCE_DIR}/lib/LAStools/LASzip/src/
    ${CMAKE_SOURCE_DIR}/lib/LAStools/LASlib/inc/
)
message("LASlib = " ${LASlib})

# Armadillo
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/armadillo)  # Use armadillo from lib
  set(ARMADILLO_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/lib/armadillo/include)
  if(WIN32)
	set(ARMADILLO_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/lib/armadillo/Release/armadillo.lib)
	
  else()
	set(ARMADILLO_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/lib/armadillo/libarmadillo.so)
  endif()
else()  # Try to find already installed armadillo
  find_package(Armadillo REQUIRED)
endif()
if(LAPACK_LIB)
  # Add custom lapack library to armadillo if specified
  set(ARMADILLO_LIBRARIES ${ARMADILLO_LIBRARIES} ${LAPACK_LIB})
endif()
include_directories(${ARMADILLO_INCLUDE_DIRS})
message("Armadillo include: " ${ARMADILLO_INCLUDE_DIRS})
message("Armadillo libraries: " ${ARMADILLO_LIBRARIES})

# ---  PYTHON BINDINGS  --- #
# ------------------------- #
if(${PYTHON_BINDING})
  # Set PYTHON_BINDING to compile boost with python binding support
  find_package(Boost 1.71.0 REQUIRED COMPONENTS
    ${BASE_BOOST_COMPONENTS} python${PYTHON_VERSION}
  )
  # Add PYTHON_BINDING define to compile binding code
  add_definitions(-DPYTHON_BINDING)
  add_definitions(-DBOOST_PYTHON_STATIC_LIB)
else()
  find_package(Boost 1.71.0 REQUIRED COMPONENTS
    ${BASE_BOOST_COMPONENTS}
  )
endif()
if(Boost_FOUND)
  message(STATUS "boost found")
  message("Boost_INCLUDE_DIRS: " ${Boost_INCLUDE_DIRS})
  include_directories(${Boost_INCLUDE_DIRS})
  message("Boost_LIBRARIES: " ${Boost_LIBRARIES})
endif()




# ---  C O M P I L A T I O N  --- #
# ------------------------------- #
ADD_EXECUTABLE(helios ${sources} AppIcon.rc)
target_link_libraries(helios ${LASlib})

if(PYTHON_BINDING)
  set(PYTHON_TARGET_LINK ${PYTHON_LIBRARIES})
  if(WIN32)
	ADD_LIBRARY(pyhelios SHARED ${sources} ${pysources})
	set_target_properties(pyhelios PROPERTIES SUFFIX ".pyd")
  else()
	ADD_LIBRARY(libhelios SHARED ${sources})
	set_target_properties(libhelios PROPERTIES PREFIX "")
    set_target_properties(libhelios PROPERTIES SUFFIX ".so")
	ADD_LIBRARY(pyhelios SHARED ${pysources})
	set_target_properties(pyhelios PROPERTIES SUFFIX ".so")
  endif()
  set_target_properties(pyhelios PROPERTIES PREFIX "")
  set_target_properties(pyhelios PROPERTIES OUTPUT_NAME "pyhelios")
  set_target_properties(pyhelios PROPERTIES POSITION_INDEPENDENT_CODE True)
  message("PYTHON_TARGET_LINK = ${PYTHON_TARGET_LINK}")
endif()
if(WIN32 OR MSVC)  # Windows compilation
  target_link_libraries(
      helios
      ${Boost_LIBRARIES}
      ${LASlib}
      ${ARMADILLO_LIBRARIES}
      ${PYTHON_TARGET_LINK}
      ${GDAL_LIBRARIES}
      ${PYTHON_TARGET_LINK}
  )
  if(PYTHON_BINDING)
    target_link_libraries(
        pyhelios
        ${Boost_LIBRARIES}
        ${LASlib}
        ${ARMADILLO_LIBRARIES}
        ${PYTHON_TARGET_LINK}
		${GDAL_LIBRARIES}
		${PYTHON_TARGET_LINK}
    )
   endif()
else()  # Linux compilation
  target_link_libraries(
      helios
      ${Boost_LIBRARIES}
      ${LASlib}
      ${ARMADILLO_LIBRARIES}
      ${PYTHON_TARGET_LINK}
      ${GDAL_LIBRARIES}
      z
      ${PYTHON_TARGET_LINK}
  )
  if(PYTHON_BINDING)
    target_link_libraries(
        libhelios
        ${Boost_LIBRARIES}
        ${LASlib}
        ${ARMADILLO_LIBRARIES}
        ${PYTHON_TARGET_LINK}
        ${GDAL_LIBRARIES}
        z
    )
    target_link_libraries(
        pyhelios
        libhelios
    )
  endif()
endif()

##############################################
