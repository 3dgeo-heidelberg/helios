#if DATA_ANALYTICS >= 2
#include <helios/dataanalytics/HDA_GlobalVars.h>

namespace helios {
namespace analytics {

// ***  GLOBAL OBJECT  *** //
// *********************** //
HDA_GlobalVars HDA_GV;

// ***  WRITE METHODS  *** //
// *********************** //
HDA_GlobalVars&
HDA_GlobalVars::incrementGeneratedSubraysCount()
{
  std::unique_lock<std::mutex> lock(generatedSubraysCount_mutex);
  ++generatedSubraysCount;
  return *this;
}

HDA_GlobalVars&
HDA_GlobalVars::incrementGeneratedRaysBeforeEarlyAbortCount()
{
  std::unique_lock<std::mutex> lock(generatedRaysBeforeEarlyAbortCount_mutex);
  ++generatedRaysBeforeEarlyAbortCount;
  return *this;
}

HDA_GlobalVars&
HDA_GlobalVars::incrementGeneratedRaysAfterEarlyAbortCount()
{
  std::unique_lock<std::mutex> lock(generatedRaysAfterEarlyAbortCount_mutex);
  ++generatedRaysAfterEarlyAbortCount;
  return *this;
}

HDA_GlobalVars&
HDA_GlobalVars::incrementIntensityComputationsCount()
{
  std::unique_lock<std::mutex> lock(intensityComputationsCount_mutex);
  ++intensityComputationsCount;
  return *this;
}

HDA_GlobalVars&
HDA_GlobalVars::incrementIntersectiveSubraysCount()
{
  std::unique_lock<std::mutex> lock(intersectiveSubraysCount_mutex);
  ++intersectiveSubraysCount;
  return *this;
}

HDA_GlobalVars&
HDA_GlobalVars::incrementNonIntersectiveSubraysCount()
{
  std::unique_lock<std::mutex> lock(nonIntersectiveSubraysCount_mutex);
  ++nonIntersectiveSubraysCount;
  return *this;
}

HDA_GlobalVars&
HDA_GlobalVars::incrementNonIntersectiveSubraysDueToNullTimeCount()
{
  std::unique_lock<std::mutex> lock(
    nonIntersectiveSubraysDueToNullTimeCount_mutex);
  ++nonIntersectiveSubraysDueToNullTimeCount;
  return *this;
}

HDA_GlobalVars&
HDA_GlobalVars::incrementSubrayIntersectionCount()
{
  std::unique_lock<std::mutex> lock(subrayIntersectionCount_mutex);
  ++subrayIntersectionCount;
  return *this;
}

HDA_GlobalVars&
HDA_GlobalVars::incrementSubrayNonIntersectionCount()
{
  std::unique_lock<std::mutex> lock(subrayNonIntersectionCount_mutex);
  ++subrayNonIntersectionCount;
  return *this;
}

// ***  READ METHODS  *** //
// ********************** //
std::size_t
HDA_GlobalVars::getGeneratedSubraysCount()
{
  std::unique_lock<std::mutex> lock(generatedSubraysCount_mutex);
  return generatedSubraysCount;
}

std::size_t
HDA_GlobalVars::getGeneratedRaysBeforeEarlyAbortCount()
{
  std::unique_lock<std::mutex> lock(generatedRaysBeforeEarlyAbortCount_mutex);
  return generatedRaysBeforeEarlyAbortCount;
}

std::size_t
HDA_GlobalVars::getGeneratedRaysAfterEarlyAbortCount()
{
  std::unique_lock<std::mutex> lock(generatedRaysAfterEarlyAbortCount_mutex);
  return generatedRaysAfterEarlyAbortCount;
}

std::size_t
HDA_GlobalVars::getIntersectiveSubraysCount()
{
  std::unique_lock<std::mutex> lock(intersectiveSubraysCount_mutex);
  return intersectiveSubraysCount;
}

std::size_t
HDA_GlobalVars::getNonIntersectiveSubraysCount()
{
  std::unique_lock<std::mutex> lock(nonIntersectiveSubraysCount_mutex);
  return nonIntersectiveSubraysCount;
}

std::size_t
HDA_GlobalVars::getNonIntersectiveSubraysDueToNullTimeCount()
{
  std::unique_lock<std::mutex> lock(
    nonIntersectiveSubraysDueToNullTimeCount_mutex);
  return nonIntersectiveSubraysDueToNullTimeCount;
}

std::size_t
HDA_GlobalVars::getSubrayIntersectionCount()
{
  std::unique_lock<std::mutex> lock(subrayIntersectionCount_mutex);
  return subrayIntersectionCount;
}

std::size_t
HDA_GlobalVars::getSubrayNonIntersectionCount()
{
  std::unique_lock<std::mutex> lock(subrayNonIntersectionCount_mutex);
  return subrayNonIntersectionCount;
}

std::size_t
HDA_GlobalVars::getIntensityComputationsCount()
{
  std::unique_lock<std::mutex> lock(intensityComputationsCount_mutex);
  return intensityComputationsCount;
}

}
}

#endif
