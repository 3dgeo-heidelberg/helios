# ---  B U I L D I N G  --- #
# ------------------------- #
ADD_EXECUTABLE(helios ${sources} AppIcon.rc)

# HANDLE PYTHON BINDING
if(PYTHON_BINDING)
    set(PYTHON_TARGET_LINK ${PYTHON_LIBRARIES})
    if(WIN32)
        ADD_LIBRARY(pyhelios SHARED ${sources} ${pysources})
        set_target_properties(pyhelios PROPERTIES SUFFIX ".pyd")
    else()
        ADD_LIBRARY(libhelios SHARED ${sources})
        set_target_properties(libhelios PROPERTIES PREFIX "")
        set_target_properties(libhelios PROPERTIES SUFFIX ".so")
        ADD_LIBRARY(pyhelios SHARED ${pysources})
        set_target_properties(pyhelios PROPERTIES SUFFIX ".so")
    endif()
    set_target_properties(pyhelios PROPERTIES PREFIX "")
    set_target_properties(pyhelios PROPERTIES OUTPUT_NAME "_pyhelios")
    set_target_properties(pyhelios PROPERTIES POSITION_INDEPENDENT_CODE True)
    message("PYTHON_TARGET_LINK = ${PYTHON_TARGET_LINK}")
endif()




# HANDLE PCL BINDING
if(PCL_BINDING)
    set(PCL_TARGET_LINK ${PCL_LIBRARIES} Eigen3::Eigen ${VTK_LIBRARIES})
    message("PCL_TARGET_LINK = ${PCL_TARGET_LINK}")
endif()




# DEFINE LIST OF ALL TARGET LIBRARIES
set(
    HELIOS_TARGET_LIBRARIES
    ${Boost_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
    ${GDAL_LIBRARIES}
    ${LASlib}
    ${PYTHON_TARGET_LINK}
    ${PCL_TARGET_LINK}
)
if(WIN32 OR MSVC)  # Handle Windows specific libraries here
else()  # Handle Linux specific libraries here
    list(APPEND HELIOS_TARGET_LIBRARIES z)
endif()




# LINK TARGET LIBRARIES
if(WIN32 OR MSVC)  # Windows compilation
    target_link_libraries(helios ${HELIOS_TARGET_LIBRARIES})
    if(PYTHON_BINDING)
        target_link_libraries(pyhelios ${HELIOS_TARGET_LIBRARIES})
    endif()
else()  # Linux compilation
    target_link_libraries(helios ${HELIOS_TARGET_LIBRARIES})
    if(PYTHON_BINDING)
        target_link_libraries(libhelios ${HELIOS_TARGET_LIBRARIES})
        target_link_libraries(pyhelios libhelios)
    endif()
endif()
