#include <XmlSceneLoader.h>
#include <XmlUtils.h>
#include <TimeWatcher.h>
#include <GeoTiffFileLoader.h>
#include <WavefrontObjFileLoader.h>
#include <XYZPointCloudFileLoader.h>
#include <DetailedVoxelLoader.h>

#include <boost/lexical_cast.hpp>

#include <logging.hpp>


std::shared_ptr<Scene>
XmlSceneLoader::createSceneFromXml(
    tinyxml2::XMLElement *sceneNode,
    std::string path
){
    std::shared_ptr<Scene> scene(new Scene());
    scene->sourceFilePath = path;

    TimeWatcher tw;

    // ####################### BEGIN Loop over all part nodes
    // ############################
    int partIndex = -1;
    tinyxml2::XMLElement *scenePartNodes = sceneNode->FirstChildElement("part");

    size_t scenePartCounter = 0;
    tw.start();
    while (scenePartNodes != nullptr) {
        partIndex++;
        ScenePart *scenePart = nullptr;
        tinyxml2::XMLElement *filterNodes =
            scenePartNodes->FirstChildElement("filter");
        bool holistic = false;
        while (filterNodes != nullptr) {
            std::string filterType = filterNodes->Attribute("type");
            std::transform(
                filterType.begin(),
                filterType.end(),
                filterType.begin(),
               ::tolower
            );
            AbstractGeometryFilter *filter = nullptr;

            // ################### BEGIN Set up filter ##################

            // Apply scale transformation:
            if (filterType == "scale") {
                filter = new ScaleFilter(scenePart);
            }

            // Read GeoTiff file:
            else if (filterType == "geotiffloader") {
                filter = new GeoTiffFileLoader();
            }

            // Read Wavefront Object file:
            else if (filterType == "objloader") {
                filter = new WavefrontObjFileLoader();
            }

            // Apply rotation filter:
            else if (filterType == "rotate") {
                filter = new RotateFilter(scenePart);
            }

            // Apply translate transformation:
            else if (filterType == "translate") {
                filter = new TranslateFilter(scenePart);
            }

            // Read xyz ASCII point cloud file:
            else if (filterType == "xyzloader") {
                // Read defined point cloud color:
                Color4f color(1.0, 1.0, 1.0, 1.0);
                if (scenePartNodes->FindAttribute("color") != nullptr) {
                    color = XmlUtils::createColorFromXml(
                        scenePartNodes->FirstChildElement("color")
                    );
                }
                filter = new XYZPointCloudFileLoader();
                holistic = true;
            }

            // Read detailed voxels file
            else if (filterType == "detailedvoxels") {
                filter = new DetailedVoxelLoader();
            }

            // ################### END Set up filter ##################

            // Finally, apply the filter:
            if (filter != nullptr) {
                // Set params:
                filter->params = XmlUtils::createParamsFromXml(filterNodes);
                logging::DEBUG("Applying filter: " + filterType);
                scenePart = filter->run();
                if (scenePart == filter->primsOut)
                    filter->primsOut = nullptr;
                delete filter;
            }

            filterNodes = filterNodes->NextSiblingElement("filter");
        }
        // ############## END Loop over filter nodes ##################

        // ######### BEGIN Read and set scene part ID #########
        std::string partId = "";
        tinyxml2::XMLAttribute const *partIdAttr =
            scenePartNodes->FindAttribute("id");
        char const *str = NULL;
        if (partIdAttr != NULL)
            str = scenePartNodes->FindAttribute("id")->Value();
        if (str != NULL) {
            partId = std::string(str);
            try {
                boost::lexical_cast<int>(partId);
            } catch (boost::bad_lexical_cast &blcex) {
                std::stringstream exss;
                exss << "You have provided a scene part id \"" << partId
                     << "\" which is non numerical.\n"
                     << "Caution! "
                     << "This is not compatible with LAS format specification"
                     << std::endl;
                logging::INFO(exss.str());
            }
        }

        bool splitPart = true;
        if (partId.empty()) {
            scenePart->mId = std::to_string(partIndex);
        } else {
            scenePart->mId = partId;
            splitPart = false;
        }

        // ######### END Read and set scene part ID #########

        // Consider scene loading specification
        scenePartCounter++;
        sceneSpec.apply(scenePart);

        // For all primitives, set reference to their scene part and transform:
        std::shared_ptr<ScenePart> sharedScenePart(scenePart);
        for (Primitive *p : scenePart->mPrimitives) {
            p->part = sharedScenePart;
            p->rotate(scenePart->mRotation);
            if (holistic) {
                for (size_t i = 0; i < p->getNumVertices(); i++) {
                    p->getVertices()[i].pos.x *= scenePart->mScale;
                    p->getVertices()[i].pos.y *= scenePart->mScale;
                    p->getVertices()[i].pos.z *= scenePart->mScale;
                }
            }
            p->scale(scenePart->mScale);
            p->translate(scenePart->mOrigin);
        }

        // Add scene part to the scene:
        scene->primitives.insert(
            scene->primitives.end(),
            scenePart->mPrimitives.begin(),
            scenePart->mPrimitives.end()
        );

        // Split subparts into different scene parts
        if (splitPart) {
            size_t partIndexOffset = scenePart->subpartLimit.size() - 1;
            if (scenePart->splitSubparts())
                partIndex += partIndexOffset;
        }
        scenePartNodes = scenePartNodes->NextSiblingElement("part");
    }
    tw.stop();

    std::stringstream ss;
    ss << std::to_string(scenePartCounter) << " sceneparts loaded in " << tw.getElapsedDecimalSeconds() << "s\n";
    logging::INFO(ss.str());
    // ####################### END Loop over all part nodes
    // ############################
    bool success = scene->finalizeLoading();

    if (!success) {
        logging::ERR("Finalizing the scene failed.");
        exit(-1);
    }

    return scene;
}
