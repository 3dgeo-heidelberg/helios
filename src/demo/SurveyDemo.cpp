#ifdef PCL_BINDING

#include <SurveyDemo.h>

#include <filesystem>

using namespace HeliosDemos;

bool
SurveyDemo::validateSurveyPath()
{
  return std::filesystem::exists(surveyPath) &&
         std::filesystem::is_regular_file(surveyPath);
}

bool
SurveyDemo::validateAssetsPath()
{
  return std::filesystem::exists(assetsPath) &&
         std::filesystem::is_directory(assetsPath);
}

#endif
