#include <DetailedVoxelLoader.h>
#include <GeoTiffFileLoader.h>
#include <KDTreeFactoryMaker.h>
#include <NullGeometryFilter.h>
#include <TimeWatcher.h>
#include <WavefrontObjFileLoader.h>
#include <XYZPointCloudFileLoader.h>
#include <XmlSceneLoader.h>
#include <XmlUtils.h>
#include <scene/dynamic/DynScene.h>

#include <logging.hpp>

#include <boost/lexical_cast.hpp>
#include <memory>

// ***  SCENE CREATION  *** //
// ************************ //
std::shared_ptr<Scene>
XmlSceneLoader::createSceneFromXml(tinyxml2::XMLElement* sceneNode,
                                   std::string path,
                                   SerialSceneWrapper::SceneType* sceneType)
{
  std::shared_ptr<StaticScene> scene = std::make_shared<StaticScene>();
  bool dynScene = false;
  scene->sourceFilePath = path;

  TimeWatcher tw;

  // Loop over all part nodes
  int partIndex = -1;
  tinyxml2::XMLElement* scenePartNode = sceneNode->FirstChildElement("part");
  size_t scenePartCounter = 0;
  tw.start();
  while (scenePartNode != nullptr) {
    partIndex++;
    bool holistic = false;
    bool dynObject = false;

    // Load filter nodes, if any
    std::shared_ptr<ScenePart> scenePart = loadFilters(scenePartNode, holistic);

    // Read and set scene part ID
    bool splitPart = loadScenePartId(scenePartNode, partIndex, scenePart);

    // Validate loaded scene part
    if (!validateScenePart(scenePart, scenePartNode)) {
      // If not valid, proceed to next scene part, ignoring the malformed
      scenePartNode = scenePartNode->NextSiblingElement("part");
      continue;
    }

    // Load dynamic motions if any
    std::shared_ptr<DynSequentiableMovingObject> dsmo =
      loadDynMotions(scenePartNode, scenePart);
    if (!dynScene && dsmo != nullptr) {
      scene = makeSceneDynamic(scene);
      dynScene = true;
    }
    if (dsmo != nullptr) { // Append to scene, replace scene part, flag dyn
      std::static_pointer_cast<DynScene>(scene)->appendDynObject(dsmo);
      scenePart = std::static_pointer_cast<ScenePart>(dsmo);
      dynObject = true;
    }

    // Consider scene loading specification
    scenePartCounter++;
    sceneSpec.apply(scenePart);

    // Digest scene part
    digestScenePart(
      scenePart, scene, holistic, splitPart, dynObject, partIndex);

    // Read next scene part from XML
    scenePartNode = scenePartNode->NextSiblingElement("part");
  }

  // Report elapsed time
  tw.stop();
  std::stringstream ss;
  ss << std::to_string(scenePartCounter) << " sceneparts loaded in "
     << tw.getElapsedDecimalSeconds() << "s\n";
  logging::TIME(ss.str());

  // Set KDGrove factory and finish scene loading
  scene->setKDGroveFactory(nullptr); // Prevent building before serializing
  bool success = scene->finalizeLoading();
  if (!success) {
    logging::ERR("Finalizing the scene failed.");
    exit(-1);
  }
  scene->setKDGroveFactory(makeKDGroveFactory()); // Better after building

  // Store scene type if requested
  if (sceneType != nullptr) {
    if (dynScene)
      *sceneType = SerialSceneWrapper::SceneType::DYNAMIC_SCENE;
    else
      *sceneType = SerialSceneWrapper::SceneType::STATIC_SCENE;
  }

  // Handle dynamic scene attributes
  if (dynScene) {
    handleDynamicSceneAttributes(sceneNode,
                                 std::static_pointer_cast<DynScene>(scene));
  }

  // Return built scene
  return scene;
}

std::shared_ptr<ScenePart>
XmlSceneLoader::loadFilters(tinyxml2::XMLElement* scenePartNode, bool& holistic)
{
  ScenePart* scenePart = nullptr;
  tinyxml2::XMLElement* filterNodes =
    scenePartNode->FirstChildElement("filter");
  while (filterNodes != nullptr) {
    // Load the filter
    std::string filterType = filterNodes->Attribute("type");
    AbstractGeometryFilter* filter =
      loadFilter(filterNodes, holistic, scenePart);
    // Apply the filter
    if (filter != nullptr) {
      // Set params:
      filter->setAssetsDir(assetsDir);
      filter->params = XmlUtils::createParamsFromXml(filterNodes);
      logging::DEBUG("Applying filter: " + filterType);
      scenePart = filter->run();
      // Load the sawps now so their baseline is defined from the raw
      // geometry with no transformation filters (e.g., scales or rotations)
      if (XmlUtils::isGeometryLoadingFilter(filter) &&
          scenePartNode->FirstChildElement("swap") != nullptr) {
        if (scenePart->sorh != nullptr) {
          std::stringstream ss;
          ss << "XmlSceneLoader::loadFilters found a geometry "
                "loading filter when a SwapOnRepeatHandler has "
                "already been built.";
          logging::ERR(ss.str());
          throw HeliosException(ss.str());
        }
        scenePart->sorh = loadScenePartSwaps(scenePartNode, scenePart);
      }
      // Delete the filter (not used anymore)
      if (scenePart == filter->primsOut)
        filter->primsOut = nullptr;
      delete filter;
    }

    filterNodes = filterNodes->NextSiblingElement("filter");
  }
  // ############## END Loop over filter nodes ##################
  return std::shared_ptr<ScenePart>(scenePart);
}

AbstractGeometryFilter*
XmlSceneLoader::loadFilter(tinyxml2::XMLElement* filterNode,
                           bool& holistic,
                           ScenePart* scenePart)
{
  std::string filterType = filterNode->Attribute("type");

  std::transform(
    filterType.begin(), filterType.end(), filterType.begin(), ::tolower);
  AbstractGeometryFilter* filter = nullptr;

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
    filter = new XYZPointCloudFileLoader();
    holistic = true;
  }

  // Read detailed voxels file
  else if (filterType == "detailedvoxels") {
    filter = new DetailedVoxelLoader();
  }
  // ################### END Set up filter ##################

  filter->setAssetsDir(assetsDir);

  return filter;
}

std::shared_ptr<SwapOnRepeatHandler>
XmlSceneLoader::loadScenePartSwaps(tinyxml2::XMLElement* scenePartNode,
                                   ScenePart* scenePart)
{
  // Find swap nodes
  tinyxml2::XMLElement* swapNodes = scenePartNode->FirstChildElement("swap");
  // If there are no swaps, then there is no handler
  if (swapNodes == nullptr)
    return nullptr;
  // Build handler from the swaps
  std::shared_ptr<SwapOnRepeatHandler> sorh =
    std::make_shared<SwapOnRepeatHandler>();
  while (swapNodes != nullptr) {
    bool holistic = false;
    int const swapStep =
      boost::get<int>(XmlUtils::getAttribute(swapNodes, "swapStep", "int", 1));
    bool const keepCRS = boost::get<bool>(
      XmlUtils::getAttribute(swapNodes, "keepCRS", "bool", sorh->isKeepCRS()));
    sorh->pushTimeToLive(swapStep);
    sorh->setKeepCRS(keepCRS);
    tinyxml2::XMLElement* filterNodes = swapNodes->FirstChildElement("filter");
    std::deque<AbstractGeometryFilter*> swapFilters;
    std::shared_ptr<ScenePart> spGeom = nullptr;
    // Handle null geometry filter
    if (XmlUtils::hasAttribute(swapNodes, "force_null")) {
      swapFilters.push_back(new NullGeometryFilter(scenePart));
    } else {
      // Add non-null filters to the handler
      while (filterNodes != nullptr) {
        AbstractGeometryFilter* filter =
          loadFilter(filterNodes, holistic, scenePart);
        filter->params = XmlUtils::createParamsFromXml(filterNodes);
        swapFilters.push_back(filter);
        // Find next XML filter element
        filterNodes = filterNodes->NextSiblingElement("filter");
      }
    }
    sorh->pushSwapFilters(swapFilters);
    // Find next XML swap element
    swapNodes = swapNodes->NextSiblingElement("swap");
  }
  sorh->prepare(scenePart);
  // Return built handler
  return sorh;
}

bool
XmlSceneLoader::loadScenePartId(tinyxml2::XMLElement* scenePartNode,
                                int partIndex,
                                std::shared_ptr<ScenePart> scenePart)
{
  std::string partId = "";
  tinyxml2::XMLAttribute const* partIdAttr = scenePartNode->FindAttribute("id");
  char const* str = nullptr;
  if (partIdAttr != nullptr)
    str = scenePartNode->FindAttribute("id")->Value();
  if (str != nullptr) {
    partId = std::string(str);
    try {
      boost::lexical_cast<int>(partId);
    } catch (boost::bad_lexical_cast& blcex) {
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

  return splitPart;
}

bool
XmlSceneLoader::validateScenePart(std::shared_ptr<ScenePart> scenePart,
                                  tinyxml2::XMLElement* scenePartNode)
{
  // If the scene part is not valid ...
  if (scenePart->getPrimitiveType() == ScenePart::PrimitiveType::NONE &&
      scenePart->mPrimitives.empty()) {
    // Obtain the path
    std::string path = "#NULL#";
    std::string pathType = "path";
    tinyxml2::XMLElement* filter = scenePartNode->FirstChildElement("filter");
    bool foundPath = false;
    while (filter != nullptr && !foundPath) {
      tinyxml2::XMLElement* param = filter->FirstChildElement("param");
      while (param != nullptr && !foundPath) {
        if (!XmlUtils::hasAttribute(param, "key"))
          continue;
        std::string key = param->Attribute("key");
        bool const isFilePath = key == "filepath";
        bool const isEFilePath = key == "efilepath";
        if (isFilePath || isEFilePath) {
          path = param->Attribute("value");
          if (isEFilePath)
            pathType = "extended path expression";
          foundPath = true;
          break;
        }
        param = param->NextSiblingElement("param");
      }
      filter = filter->NextSiblingElement("filter");
    }
    // Report the problem
    std::stringstream ss;
    ss << "XmlSceneLoader::validateScenePart detected an invalid scene "
          "part with id \""
       << scenePart->mId << "\"\n"
       << "The " << pathType << " \"" << path << "\" was given.\n"
       << "It leads to the loading of an invalid scene part, which is "
          "automatically ignored when composing the scene."
       << std::endl;
    logging::ERR(ss.str());
    return false;
  }
  return true;
}

void
XmlSceneLoader::digestScenePart(std::shared_ptr<ScenePart>& scenePart,
                                std::shared_ptr<StaticScene>& scene,
                                bool holistic,
                                bool splitPart,
                                bool dynObject,
                                int& partIndex)
{
  // For all primitives, set reference to their scene part and transform:
  ScenePart::computeTransformations(scenePart, holistic);

  // Append as static object if it is not dynamic
  // If it is dynamic, it must have been appended before
  if (!dynObject)
    scene->appendStaticObject(scenePart);

  // Add scene part primitives to the scene
  scene->primitives.insert(scene->primitives.end(),
                           scenePart->mPrimitives.begin(),
                           scenePart->mPrimitives.end());

  // Split subparts into different scene parts
  if (splitPart) {
    size_t partIndexOffset = scenePart->subpartLimit.size() - 1;
    if (scenePart->splitSubparts())
      partIndex += partIndexOffset;
  }

  // Infer type of primitive for the scene part
  size_t const numVertices = scenePart->mPrimitives[0]->getNumVertices();
  if (numVertices == 3)
    scenePart->primitiveType = ScenePart::TRIANGLE;
  else
    scenePart->primitiveType = ScenePart::VOXEL;
}

std::shared_ptr<KDTreeFactory>
XmlSceneLoader::makeKDTreeFactory()
{
  if (kdtNumJobs == 0)
    kdtNumJobs = std::thread::hardware_concurrency();
  if (kdtGeomJobs == 0)
    kdtGeomJobs = kdtNumJobs;

  if (kdtFactoryType == 1) { // Simple
    logging::DEBUG("XmlSceneLoader is using a SimpleKDTreeFactory");
    if (kdtNumJobs > 1) {
      return KDTreeFactoryMaker::makeSimpleMultiThread(kdtNumJobs, kdtGeomJobs);
    }
    return KDTreeFactoryMaker::makeSimple();
  } else if (kdtFactoryType == 2) { // SAH
    logging::DEBUG("XmlSceneLoader is using a SAHKDTreeFactory");
    if (kdtNumJobs > 1) {
      return KDTreeFactoryMaker::makeSAHMultiThread(
        kdtSAHLossNodes, kdtNumJobs, kdtGeomJobs);
    }
    return KDTreeFactoryMaker::makeSAH(kdtSAHLossNodes);
  } else if (kdtFactoryType == 3) { // Axis SAH
    logging::DEBUG("XmlSceneLoader is using a AxisSAHKDTreeFactory");
    if (kdtNumJobs > 1) {
      return KDTreeFactoryMaker::makeAxisSAHMultiThread(
        kdtSAHLossNodes, kdtNumJobs, kdtGeomJobs);
    }
    return KDTreeFactoryMaker::makeAxisSAH(kdtSAHLossNodes);
  } else if (kdtFactoryType == 4) { // Fast SAH
    logging::DEBUG("XmlSceneLoader is using a FastSAHKDTreeFactory");
    if (kdtNumJobs > 1) {
      return KDTreeFactoryMaker::makeFastSAHMultiThread(
        kdtSAHLossNodes, kdtNumJobs, kdtGeomJobs);
    }
    return KDTreeFactoryMaker::makeFastSAH(kdtSAHLossNodes);
  } else {
    std::stringstream ss;
    ss << "Unexpected KDT factory type at "
       << "XmlSceneLoader::makeKDTreeFactory: " << kdtFactoryType;
    throw HeliosException(ss.str());
  }
}

std::shared_ptr<KDGroveFactory>
XmlSceneLoader::makeKDGroveFactory()
{
  return std::make_shared<KDGroveFactory>(makeKDTreeFactory());
}

// ***  DYNAMIC SCENE LOADINGMETHODS  *** //
// ************************************** //
std::shared_ptr<DynSequentiableMovingObject>
XmlSceneLoader::loadDynMotions(tinyxml2::XMLElement* scenePartNode,
                               std::shared_ptr<ScenePart> scenePart)
{
  // Find first dmotion node
  tinyxml2::XMLElement* dmotionNode =
    scenePartNode->FirstChildElement("dmotion");
  if (dmotionNode == nullptr)
    return nullptr; // No dmotion found

  // Build the basis of dynamic sequential moving object from scene part
  std::shared_ptr<DynSequentiableMovingObject> dsmo =
    std::make_shared<DynSequentiableMovingObject>(*scenePart, true);

  // Complete building of dynamic sequential moving object from XML
  while (dmotionNode != nullptr) {
    // Optional attributes
    std::string nextId = "";
    tinyxml2::XMLAttribute const* nextIdAttr =
      dmotionNode->FindAttribute("next");
    if (nextIdAttr != nullptr)
      nextId = nextIdAttr->Value();

    // Mandatory attributes
    tinyxml2::XMLAttribute const* idAttr = dmotionNode->FindAttribute("id");
    if (idAttr == nullptr)
      throw HeliosException(
        "XmlSceneLoader::loadDynMotions found a dmotion element with "
        "no id");
    tinyxml2::XMLAttribute const* loopAttr = dmotionNode->FindAttribute("loop");
    if (loopAttr == nullptr)
      throw HeliosException(
        "XmlSceneLoader::loadDynMotions found a dmotion element with "
        "no loop");

    // Add dynamic motion sequence
    std::shared_ptr<DynSequence<DynMotion>> dmSequence =
      std::make_shared<DynSequence<DynMotion>>(
        idAttr->Value(), nextId, loopAttr->Int64Value());
    dmSequence->append(XmlUtils::createDynMotionsVector(dmotionNode));
    if (dmSequence == nullptr) { // Check dmSequence is not null
      throw HeliosException(
        "XmlSceneLoader::loadDynMotions found a dmotion element with"
        " no motions. This is not allowed.");
    }
    dsmo->addSequence(dmSequence);

    // Next dynamic motion sequence
    dmotionNode = dmotionNode->NextSiblingElement("dmotion");
  }

  // Update scene part for each primitive so it is the new DMO
  for (Primitive* primitive : dsmo->mPrimitives) {
    primitive->part = dsmo;
  }

  // Use scene part ID to build dynamic sequentiable moving object ID
  std::stringstream ss;
  ss << "DSMO_" << scenePart->mId;
  dsmo->setId(ss.str());

  // Configure the step interval for the dynamic object and its observer
  dsmo->setStepInterval(boost::get<int>(
    XmlUtils::getAttribute(scenePartNode, "dynStep", "int", 1)));
  dsmo->setObserverStepInterval(boost::get<int>(
    XmlUtils::getAttribute(scenePartNode, "kdtDynStep", "int", 1)));

  // Get the time step for the dyn. obj. and its observer (nan if not given)
  dsmo->setDynTimeStep(boost::get<double>(
    XmlUtils::getAttribute(scenePartNode,
                           "dynTimeStep",
                           "double",
                           std::numeric_limits<double>::quiet_NaN())));
  dsmo->setObserverDynTimeStep(boost::get<double>(
    XmlUtils::getAttribute(scenePartNode,
                           "kdtDynTimeStep",
                           "double",
                           std::numeric_limits<double>::quiet_NaN())));

  // Return
  return dsmo;
}

std::shared_ptr<StaticScene>
XmlSceneLoader::makeSceneDynamic(std::shared_ptr<StaticScene> scene)
{
  // Upgrade static scene to dynamic scene
  std::shared_ptr<StaticScene> newScene =
    std::static_pointer_cast<StaticScene>(std::make_shared<DynScene>(*scene));

  // Primitives were automatically updated at scene level
  // Now, update them at static object level
  // It is as simple as rebuilding the static objects because at this point
  // there is no dynamic object in the scene yet, since it didnt support them
  newScene->clearStaticObjects();
  std::set<std::shared_ptr<ScenePart>> parts; // Scene parts with no repeats
  std::vector<Primitive*> const primitives = newScene->primitives;
  for (Primitive* primitive : primitives)
    if (primitive->part != nullptr)
      parts.insert(primitive->part);
  for (std::shared_ptr<ScenePart> part : parts)
    newScene->appendStaticObject(part);

  // Return upgraded scene
  return newScene;
}

void
XmlSceneLoader::handleDynamicSceneAttributes(tinyxml2::XMLElement* sceneNode,
                                             std::shared_ptr<DynScene> scene)
{
  // Configure step interval
  scene->setStepInterval(
    boost::get<int>(XmlUtils::getAttribute(sceneNode, "dynStep", "int", 1)));
  // Get the dynamic time step for the dynamic scene (nan if not given)
  scene->setDynTimeStep(boost::get<double>(
    XmlUtils::getAttribute(sceneNode,
                           "dynTimeStep",
                           "double",
                           std::numeric_limits<double>::quiet_NaN())));
  // Apply automatic CRS translation when requested
  glm::dvec3 const& shift = scene->getBBoxCRS()->getCentroid();
  size_t const numDynObjects = scene->numDynObjects();
  for (size_t i = 0; i < numDynObjects; ++i) {
    std::shared_ptr<DynSequentiableMovingObject> dsmo =
      std::dynamic_pointer_cast<DynSequentiableMovingObject>(
        scene->getDynObject(i));
    if (dsmo != nullptr) {
      dsmo->applyAutoCRS(shift.x, shift.y, shift.z);
    }
  }
}
