if(HELIOS_PCL)
  target_include_directories(
    helios
    PUBLIC
    "."
  )
  
  target_sources(
    helios
    PRIVATE
    "DemoSelector.cpp"
    "DynamicSceneDemo.cpp"
    "RaycastingDemo.cpp"
    "SimplePrimitivesDemo.cpp"
    "SurveyDemo.cpp"
  )
endif()
