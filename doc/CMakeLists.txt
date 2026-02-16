find_package(Doxygen REQUIRED)

# Set Doxygen options otherwise set through the Doxyfile
set(DOXYGEN_EXCLUDE_PATTERNS "${CMAKE_SOURCE_DIR}/build/*" "${CMAKE_SOURCE_DIR}/build2/*" "${CMAKE_SOURCE_DIR}/py4dgeo/*")
set(DOXYGEN_SHORT_NAMES YES)
set(DOXYGEN_GENERATE_XML YES)
set(DOXYGEN_FILE_PATTERNS "*.h" "*.hpp")
doxygen_add_docs(
    doxygen
    ${CMAKE_SOURCE_DIR}/src
    COMMENT "Generate Doxygen documentation"
)

add_custom_target(
  sphinx-doc
  COMMAND
    sphinx-build -b html
    -Dbreathe_projects.helios="${CMAKE_CURRENT_BINARY_DIR}/xml" -c
    ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/sphinx
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Generating documentation with Sphinx..."
)

add_dependencies(sphinx-doc doxygen)
