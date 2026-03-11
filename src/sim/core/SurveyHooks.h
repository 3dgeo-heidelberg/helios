#pragma once

#include <Measurement.h>
#include <Trajectory.h>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

enum class HookPoint
{
  LEG_START,
  LEG_END,
  SIM_TIME_ONCE,
  SIM_TIME_PERIODIC
};

enum class HookPayload
{
  METADATA_ONLY,
  SINCE_LAST,
  ALL_POINTS
};

enum class HookEndOfLegPolicy
{
  NONE,
  FLUSH,
  FLUSH_AND_RESET
};

struct HookContext
{
  HookPoint point = HookPoint::LEG_START;
  uint64_t stepIndex = 0;
  uint32_t playIndex = 0;
  int legIndex = 0;
  double simTime_s = 0.0;
  double gpsTime_s = 0.0;
  std::string outputPath;
  size_t totalPoints = 0;
  size_t payloadPoints = 0;
  size_t totalTrajectories = 0;
  size_t payloadTrajectories = 0;
  double scheduledTime_s = 0.0;
  double lag_s = 0.0;
};

using SurveyHookFn =
  std::function<void(HookContext const&,
                     std::vector<Measurement> const* pointsPayload,
                     std::vector<Trajectory> const* trajectoriesPayload)>;

struct SurveyHookRegistration
{
  HookPoint point = HookPoint::LEG_START;
  HookPayload payload = HookPayload::METADATA_ONLY;
  HookEndOfLegPolicy endOfLegPolicy = HookEndOfLegPolicy::FLUSH;
  bool barrier = false;
  SurveyHookFn fn;

  // time-trigger config/state
  double simTime_s = 0.0;
  double period_s = 0.0;
  bool firedOnce = false;
  double nextFire_s = 0.0;

  // payload cursor state (for SINCE_LAST)
  size_t lastMeasurementIndex = 0;
  size_t lastTrajectoryIndex = 0;
};
