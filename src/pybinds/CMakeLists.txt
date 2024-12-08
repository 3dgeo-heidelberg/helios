if(BUILD_PYTHON)
  target_include_directories(
    _pyhelios
    PUBLIC
    "."
  )

  target_sources(
    _pyhelios
    PRIVATE
    "PySceneWrapper.cpp"
    "PyHeliosSimulation.cpp"
    "PyHelios.cpp"
    "PyScenePartWrapper.cpp"
    "PyScannerWrapper.cpp"
  )
endif()
