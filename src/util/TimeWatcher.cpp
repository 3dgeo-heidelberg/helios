#include <TimeWatcher.h>
#include <iomanip>
#include <logging.hpp>
#include <sstream>

// *** CONSTRUCTOR *** //
// ******************* //
TimeWatcher::TimeWatcher()
{
  tStart = nullptr;
  tEnd = nullptr;
}

// *** PUBLIC METHODS *** //
// ********************** //
void
TimeWatcher::start()
{
  tStart = std::unique_ptr<std::chrono::high_resolution_clock::time_point>(
    new std::chrono::high_resolution_clock::time_point(
      std::chrono::high_resolution_clock::now()));
}
void
TimeWatcher::startIfNull()
{
  if (tStart == nullptr)
    start();
}
void
TimeWatcher::releaseStart()
{
  tStart = nullptr;
}
bool
TimeWatcher::hasStarted() const
{
  return tStart != nullptr;
}

void
TimeWatcher::stop()
{
  tEnd = std::unique_ptr<std::chrono::high_resolution_clock::time_point>(
    new std::chrono::high_resolution_clock::time_point(
      std::chrono::high_resolution_clock::now()));
}

void
TimeWatcher::saveStart()
{
  syncTimePoints(savedStart, tStart);
}
void
TimeWatcher::loadStart()
{
  syncTimePoints(tStart, savedStart);
}

std::shared_ptr<std::chrono::high_resolution_clock::duration>
TimeWatcher::getElapsedTime()
{
  if (hasNulls())
    return nullptr;
  return std::shared_ptr<std::chrono::high_resolution_clock::duration>(
    new std::chrono::high_resolution_clock::duration(*tEnd - *tStart));
}
double
TimeWatcher::getElapsedDecimalSeconds()
{
  return ((double)getElapsedMillis()) / 1000.0;
}
long
TimeWatcher::getElapsedSeconds()
{
  if (hasNulls())
    return 0L;
  return std::chrono::duration_cast<std::chrono::seconds>(*getElapsedTime())
    .count();
}
long
TimeWatcher::getElapsedMillis()
{
  if (hasNulls())
    return 0L;
  return std::chrono::duration_cast<std::chrono::milliseconds>(
           *getElapsedTime())
    .count();
}
long
TimeWatcher::getElapsedNanos()
{
  if (hasNulls())
    return 0L;
  return std::chrono::duration_cast<std::chrono::nanoseconds>(*getElapsedTime())
    .count();
}

std::string
TimeWatcher::getElapsedFormat()
{
  long seconds = getElapsedSeconds();
  long minutes = seconds / 60;
  long hours = minutes / 60;
  minutes = minutes % 60;
  seconds = seconds % 60;
  std::stringstream strm;
  strm << hours << ":";
  strm << std::setfill('0') << std::setw(2);
  strm << minutes << ":" << seconds;
  return strm.str();
}

void
TimeWatcher::reportSeconds(std::string msg)
{
  std::stringstream ss;
  ss << msg << getElapsedSeconds();
  logging::INFO(ss.str());
}

void
TimeWatcher::reportMillis(std::string msg)
{
  std::stringstream ss;
  ss << msg << getElapsedMillis() << std::endl;
  logging::INFO(ss.str());
}

void
TimeWatcher::reportFormat(std::string msg)
{
  std::stringstream ss;
  ss << msg << getElapsedFormat() << std::endl;
  logging::INFO(ss.str());
}

void
TimeWatcher::synchronize(TimeWatcher const& source)
{
  syncTimePoints(tStart, source.tStart);
  syncTimePoints(tEnd, source.tEnd);
  syncTimePoints(savedStart, source.savedStart);
}

// *** PRIVATE METHODS *** //
// ********************** //
bool
TimeWatcher::hasNulls()
{
  return tStart == nullptr || tEnd == nullptr;
}

void
TimeWatcher::syncTimePoints(
  std::unique_ptr<std::chrono::high_resolution_clock::time_point>& p,
  std::unique_ptr<std::chrono::high_resolution_clock::time_point> const& q)
{
  if (q == nullptr)
    p = nullptr;
  else {
    if (p == nullptr) {
      p = std::unique_ptr<std::chrono::high_resolution_clock::time_point>(
        new std::chrono::high_resolution_clock::time_point());
    }
    *p = *q;
  }
}
