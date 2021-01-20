#include <iostream>

#include <boost/variant/get.hpp>

using namespace std;

#include "TranslateFilter.h"

ScenePart* TranslateFilter::run() {
	if (primsOut == nullptr) return nullptr;

	glm::dvec3 offset = localTranslation;
	if(!useLocalTranslation) {
        offset = boost::get<glm::dvec3>(params["offset"]);
    }
    primsOut->mOrigin = offset;

	return primsOut;
}