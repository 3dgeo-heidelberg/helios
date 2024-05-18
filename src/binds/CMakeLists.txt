if(BUILD_PYTHON)
  target_include_directories(
    _helios
    PUBLIC
    ${CMAKE_SOURCE_DIR}/src
  )
endif()