# ---  C O N F I G U R A T I O N  --- #
# ----------------------------------- #
# Setup compiler flags
# Default is RELWITHDEBINFO
# Change on command line e.g. with "-DCMAKE_BUILD_TYPE=RELEASE"

# Configure flags
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RELEASE)
endif()
# Common flags
if(PCL_BINDING)
    set(CMAKE_CXX_STANDARD 14)
else()
    set(CMAKE_CXX_STANDARD 11)
endif()
if(WIN32 OR MSVC) # Windows flags
    set(CMAKE_CXX_FLAGS_RELEASE "/MD /O2 /Ob2 /DNDEBUG")
    set(CMAKE_CXX_FLAGS_DEBUG "")
    set(CMAKE_CXX_FLAGS "-D_OS_WINDOWS_  /EHsc")
else() # Linux flags
    #set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG -Wall")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -Wall")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall")
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
