if(HELIOS_PCL)
  
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
