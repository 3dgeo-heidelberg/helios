target_include_directories(
  helios
  PUBLIC
  "."
  "threadpool"
  "logger"
)

target_sources(
  helios
  PRIVATE
  "PrintUtils.cpp"
  "DateTimeUtils.cpp"
  "threadpool/SharedTaskSequencer.cpp"
  "logger/logger_core.cpp"
  "TimeWatcher.cpp"
  "helios_version.cpp"
)
