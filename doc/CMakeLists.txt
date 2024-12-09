find_package(Doxygen REQUIRED)

# Set Doxygen options otherwise set through the Doxyfile
set(DOXYGEN_SHORT_NAMES YES)
doxygen_add_docs(
    doxygen
    ${CMAKE_SOURCE_DIR}
    COMMENT "Generate Doxygen documentation"
)
