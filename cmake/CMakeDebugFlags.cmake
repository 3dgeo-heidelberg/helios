# ---  DEBUG FLAGS  --- #
# --------------------- #
#-DBUDDING_METRICS=1 to enable
if(${BUDDING_METRICS})
    add_definitions(-DBUDDING_METRICS)
endif()

#-DDATA_ANALYTICS=1 to enable
if(${DATA_ANALYTICS})
    add_definitions(-DDATA_ANALYTICS)
endif()