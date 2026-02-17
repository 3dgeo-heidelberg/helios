if(HELIOS_PCL)

  target_sources(
    helios
    PRIVATE
    "VHRaycastingCanvas.cpp"
    "VHSceneCanvas.cpp"
    "VHNormalsCanvas.cpp"
    "adapters/VHDynObjectAdapter.cpp"
    "adapters/VHStaticObjectXYZAdapter.cpp"
    "adapters/VHScannerAdapter.cpp"
    "adapters/VHStaticObjectAdapter.cpp"
    "adapters/VHStaticObjectXYZRGBAdapter.cpp"
    "VHSimpleCanvas.cpp"
    "VHCanvas.cpp"
    "VHDynCanvas.cpp"
  )
endif()
