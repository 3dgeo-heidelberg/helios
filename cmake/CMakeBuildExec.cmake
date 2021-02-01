# ---  B U I L D I N G  --- #
# ------------------------- #
ADD_EXECUTABLE(helios ${sources} AppIcon.rc)
target_link_libraries(helios ${LASlib})

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
    set_target_properties(pyhelios PROPERTIES OUTPUT_NAME "pyhelios")
    set_target_properties(pyhelios PROPERTIES POSITION_INDEPENDENT_CODE True)
    message("PYTHON_TARGET_LINK = ${PYTHON_TARGET_LINK}")
endif()
if(WIN32 OR MSVC)  # Windows compilation
    target_link_libraries(
        helios
        ${Boost_LIBRARIES}
        ${LASlib}
        ${ARMADILLO_LIBRARIES}
        ${PYTHON_TARGET_LINK}
        ${GDAL_LIBRARIES}
        ${PYTHON_TARGET_LINK}
    )
    if(PYTHON_BINDING)
        target_link_libraries(
            pyhelios
            ${Boost_LIBRARIES}
            ${LASlib}
            ${ARMADILLO_LIBRARIES}
            ${PYTHON_TARGET_LINK}
            ${GDAL_LIBRARIES}
            ${PYTHON_TARGET_LINK}
        )
    endif()
else()  # Linux compilation
    target_link_libraries(
        helios
        ${Boost_LIBRARIES}
        ${LASlib}
        ${ARMADILLO_LIBRARIES}
        ${PYTHON_TARGET_LINK}
        ${GDAL_LIBRARIES}
        z
        ${PYTHON_TARGET_LINK}
    )
    if(PYTHON_BINDING)
        target_link_libraries(
            libhelios
            ${Boost_LIBRARIES}
            ${LASlib}
            ${ARMADILLO_LIBRARIES}
            ${PYTHON_TARGET_LINK}
            ${GDAL_LIBRARIES}
            z
        )
        target_link_libraries(
            pyhelios
            libhelios
        )
    endif()
endif()
