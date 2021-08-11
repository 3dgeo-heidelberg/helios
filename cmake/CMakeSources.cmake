# ---  S O U R C E S  --- #
# ----------------------- #
# Define helios executable ###################
file(GLOB_RECURSE sources CONFIGURE_DEPENDS src/*.cpp src/*.hpp src/*.h)

# Common Helios include directories
set(HELIOS_INCLUDE_DIRECTORIES
    "src/"
    "src/assetloading/"
    "src/assetloading/geometryfilter/"
    "src/platform/"
    "src/scanner/"
    "src/scanner/beamDeflector/"
    "src/scanner/detector/"
    "src/scene/"
    "src/scene/primitives/"
    "src/scene/dynamic/"
    "src/surveyplayback/"
    "src/noise/"
    "src/maths/"
    "src/maths/rigidmotion/"
    "src/util/"
    "src/util/logger/"
    "src/test/"

)
# Pyhelios specific include directories
if(${PYTHON_BINDING})
    list(APPEND HELIOS_INCLUDE_DIRECTORIES "src/pybinds/")
endif()
# PCL specific include directories
if(${PCL_BINDING})
    list(APPEND HELIOS_INCLUDE_DIRECTORIES "src/demo/")
endif()
include_directories(${HELIOS_INCLUDE_DIRECTORIES})
