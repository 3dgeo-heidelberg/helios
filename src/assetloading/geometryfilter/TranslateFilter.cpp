#include <iostream>

#include <boost/variant/get.hpp>

using namespace std;

#include "TranslateFilter.h"

ScenePart* TranslateFilter::run() {
	if (primsOut == nullptr) return nullptr;

	// Handle translation itself
	glm::dvec3 offset = localTranslation;
	if(!useLocalTranslation) offset = boost::get<glm::dvec3>(params["offset"]);
    primsOut->mOrigin = offset;

    // Handle on ground
    if(params.find("onGround") != params.end()){
        primsOut->forceOnGround = boost::get<int>(params["onGround"]);
    }

    // Return
	return primsOut;
}