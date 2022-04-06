# ---  S O U R C E S  --- #
# ----------------------- #
# Define helios executable ###################
file(GLOB_RECURSE sources CONFIGURE_DEPENDS src/*.cpp src/*.hpp src/*.h)

# Common Helios include directories
set(HELIOS_INCLUDE_DIRECTORIES
    "src/"
    "src/adt/"
    "src/adt/tree/"
    "src/adt/bintree/"
    "src/adt/kdtree/"
    "src/adt/grove/"
    "src/alg/"
    "src/alg/raycast/"
    "src/assetloading/"
    "src/assetloading/geometryfilter/"
    "src/filems/"
    "src/filems/facade/"
    "src/filems/factory/"
    "src/filems/read/"
    "src/filems/serialization/"
    "src/filems/util/"
    "src/filems/write/"
    "src/filems/write/core/"
    "src/filems/write/comps/"
    "src/filems/write/strategies/"
    "src/main/"
    "src/platform/"
    "src/scanner/"
    "src/scanner/beamDeflector/"
    "src/scanner/detector/"
    "src/scene/"
    "src/scene/primitives/"
    "src/scene/dynamic/"
    "src/sim/comps/"
    "src/sim/core/"
    "src/sim/tools/"
    "src/noise/"
    "src/maths/"
    "src/maths/rigidmotion/"
    "src/maths/fluxionum/"
    "src/surfaceinspector/maths/"
    "src/surfaceinspector/maths/functions/"
    "src/surfaceinspector/maths/permuters/"
    "src/surfaceinspector/util/"
    "src/surfaceinspector/util/draggers/"
    "src/util/"
    "src/util/threadpool/"
    "src/util/serialization/"
    "src/util/logger/"
    "src/hpc/"
    "src/test/"
)
# Pyhelios specific include directories
if(${PYTHON_BINDING})
    list(APPEND HELIOS_INCLUDE_DIRECTORIES "src/pybinds/")
endif()
# PCL specific include directories
if(${PCL_BINDING})
    list(APPEND HELIOS_INCLUDE_DIRECTORIES
        "src/visualhelios/"
        "src/visualhelios/adapters/"
        "src/demo/"
    )
endif()
include_directories(${HELIOS_INCLUDE_DIRECTORIES})
