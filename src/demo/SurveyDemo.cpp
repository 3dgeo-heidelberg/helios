#ifdef PCL_BINDING

#include <SurveyDemo.h>

#include <boost/filesystem.hpp>

using namespace HeliosDemos;

bool SurveyDemo::validateSurveyPath(){
    return  boost::filesystem::exists(surveyPath) &&
            boost::filesystem::is_regular_file(surveyPath);
}

bool SurveyDemo::validateAssetsPath(){
    return  boost::filesystem::exists(assetsPath) &&
            boost::filesystem::is_directory(assetsPath);
}

#endif
