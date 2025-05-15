if(HELIOS_HEADERCHECK)
  file(GLOB_RECURSE helios_headers
    "${PROJECT_SOURCE_DIR}/src/*.h"
    "${PROJECT_SOURCE_DIR}/src/*.hpp"
  )

  foreach(header ${helios_headers})
    get_filename_component(header_name ${header} NAME)
    string(REPLACE ".h" "" header_name ${header_name})
    string(REPLACE ".hpp" "" header_name ${header_name})
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/template.cpp.in
      ${CMAKE_CURRENT_BINARY_DIR}/${header_name}.headercheck.cpp
      @ONLY
    )
    message(STATUS "Generating headercheck for ${header_name}: ${header_name}.headercheck")
    add_executable(${header_name}_headercheck
      ${CMAKE_CURRENT_BINARY_DIR}/${header_name}.headercheck.cpp
    )
    target_link_libraries(${header_name}_headercheck PUBLIC helios)
  endforeach()
endif()