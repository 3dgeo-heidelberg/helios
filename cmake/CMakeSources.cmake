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
    "src/maths/rigidmotion/"
    "src/util/"
    "src/util/logger/"
    "src/test/"
)
