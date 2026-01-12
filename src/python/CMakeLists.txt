if(BUILD_PYTHON)
  target_include_directories(_helios PUBLIC ${CMAKE_SOURCE_DIR}/src)

  target_sources(_helios PRIVATE "PyHeliosSimulation.cpp" "PyXMLReader.cpp" "SceneHandling.cpp" "InterpolatedPlatformPreparation.cpp" "SurveyHandling.cpp")
endif()
