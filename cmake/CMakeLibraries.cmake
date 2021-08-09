# ---  L I B R A R I E S  --- #
# --------------------------- #
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
else()
    set(GDAL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/gdal/include")
    set(GDAL_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/lib/gdal/lib/libgdal.so")
endif()
find_package(GDAL 3.0)
if(NOT GDAL_FOUND)
    message("GDAL was not found on expected paths. Searching system-level ...")
    unset(GDAL_INCLUDE_DIR)
    unset(GDAL_LIBRARY)
    find_package(GDAL 3.0 REQUIRED)
endif()
if(GDAL_FOUND)
    message(STATUS "gdal found")
    message("GDAL_INCLUDE_DIRS: " ${GDAL_INCLUDE_DIRS})
    message("GDAL_LIBRARIES: " ${GDAL_LIBRARIES})
    include_directories(${GDAL_INCLUDE_DIRS})
else()
    message(WARNING "GDAL was not found")
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
