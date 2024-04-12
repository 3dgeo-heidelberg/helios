# Find the LAStools (http://www.cs.unc.edu/~isenburg/lastools/) libraries LASlib and LASzip
# This module defines:
# LASTOOLS_FOUND, if false do not try to link against LASlib
# LASTOOLS_LIBRARIES, the name of the LASlib library to link against
# LASTOOLS_INCLUDE_DIRS, the LASlib and LASzip include directories
#
# You can also specify the environment variable LASTOOLS or define it with
# -DLASTools_DIR=... to hint at the module where to search for LAStools if it's
# installed in a non-standard location.

# Find the include for LASlib
find_path(LASLIB_INCLUDE_DIR lasreader.hpp
  HINTS
  $ENV{LASTools_DIR}
  ${LASTools_DIR}
  PATH_SUFFIXES
  LASlib/inc/ # Linux after making
  include/LASlib/ # after using CMake
  PATHS
  ~/Library/Frameworks
  /Library/Frameworks
  /usr/local/include/LASlib
  /usr/include/LASlib
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt
  )

find_library(LASLIB_LIBRARY_TMP NAMES las LAS LASlib.lib libLASlib.dylib
  HINTS
  $ENV{LASTools_DIR}
  ${LASTools_DIR}
  PATH_SUFFIXES
  LASlib/lib/
  LASlib/lib/RelWithDebInfo/
  lib/LASlib/ # After using CMake
  PATHS
  /sw
  /opt/local
  /opt/csw
  /opt
  )

set(LASTOOLS_FOUND FALSE)
if (LASLIB_LIBRARY_TMP)
  set(LASTOOLS_LIBRARIES ${LASLIB_LIBRARY_TMP} CACHE STRING
    "Where the LAStools LASlib library can be found")
  set(LASTOOLS_INCLUDE_DIRS ${LASLIB_INCLUDE_DIR} CACHE
    STRING "Include directories for LASlib")
  set(LASLIB_LIBRARY_TMP ${LASLIB_LIBRARY_TMP} CACHE INTERNAL "")
  set(LASTOOLS_FOUND TRUE)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LASTools REQUIRED_VARS LASTOOLS_LIBRARIES
  LASLIB_INCLUDE_DIR LASTOOLS_INCLUDE_DIRS)