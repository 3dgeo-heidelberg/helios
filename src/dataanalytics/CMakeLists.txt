if(HELIOS_DATA_ANALYTICS)
  target_include_directories(
    helios
    PUBLIC
    "."
  )

  target_sources(
    helios
    PRIVATE
    "HDA_GlobalVars.cpp"
    "HDA_PulseRecorder.cpp"
    "HDA_Recorder.cpp"
    "HDA_SimStepRecorder.cpp"
    "HDA_StateJSONReporter.cpp"
  )
endif()
