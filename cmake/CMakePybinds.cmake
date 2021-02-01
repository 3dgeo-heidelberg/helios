# ---  PYTHON BINDINGS  --- #
# ------------------------- #
if(${PYTHON_BINDING})
    # Set PYTHON_BINDING to compile boost with python binding support
    find_package(Boost 1.71.0 REQUIRED COMPONENTS
        ${BASE_BOOST_COMPONENTS} python${PYTHON_VERSION}
        )
    # Add PYTHON_BINDING define to compile binding code
    add_definitions(-DPYTHON_BINDING)
    add_definitions(-DBOOST_PYTHON_STATIC_LIB)
else()
    find_package(Boost 1.71.0 REQUIRED COMPONENTS
        ${BASE_BOOST_COMPONENTS}
        )
endif()
if(Boost_FOUND)
    message(STATUS "boost found")
    message("Boost_INCLUDE_DIRS: " ${Boost_INCLUDE_DIRS})
    include_directories(${Boost_INCLUDE_DIRS})
    message("Boost_LIBRARIES: " ${Boost_LIBRARIES})
endif()
