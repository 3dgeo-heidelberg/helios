#include <XmlUtils.h>
#include <MathConverter.h>
#include <HeliosException.h>
#include <UniformNoiseSource.h>
#include <NormalNoiseSource.h>
#include <rigidmotion/RigidMotionR3Factory.h>

#include <logging.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <armadillo>


// ***  STATIC METHODS  *** //
// ************************ //
Color4f XmlUtils::createColorFromXml(tinyxml2::XMLElement* node){
    Color4f col;
    try {
        float r = boost::lexical_cast<float>(node->Attribute("r"));
        float g = boost::lexical_cast<float>(node->Attribute("g"));
        float b = boost::lexical_cast<float>(node->Attribute("b"));
        col = Color4f(r, g, b, 1);
    } catch (std::exception &e) {
        logging::INFO(std::string("Error creating color from xml.\nEXCEPTION: ") +
                      e.what());
    }
    return col;
}

std::map<std::string, ObjectT> XmlUtils::createParamsFromXml(
    tinyxml2::XMLElement* paramsNode
){
    std::map<std::string, ObjectT> result;

    if (paramsNode == nullptr) {
        return result;
    }

    tinyxml2::XMLElement *element = paramsNode->FirstChildElement("param");
    while (element != nullptr) {
        try {
            std::string type = element->Attribute("type");
            std::string key = element->Attribute("key");
            std::string valueString;
            const char *attribute = element->Attribute("value");
            if (attribute)
                valueString = attribute;

            if (type.empty() || type.compare("string") == 0) {
                result.insert(std::pair<std::string, std::string>(key, valueString));
            } else {

                if (type == "boolean") {
                    bool b = valueString == "true";
                    result.insert(std::pair<std::string, bool>(key, b));
                } else if (type == "double") {
                    result.insert(std::pair<std::string, double>(
                        key, boost::lexical_cast<double>(valueString)));
                } else if (type == "integer" || type == "int") {
                    result.insert(std::pair<std::string, int>(
                        key, boost::lexical_cast<int>(valueString)));
                } else if (type == "rotation") {
                    result.insert(std::pair<std::string, Rotation>(
                        key,
                        createRotationFromXml(element)
                    ));
                } else if (type == "vec3") {
                    std::vector<std::string> vec;
                    boost::split(vec, valueString, boost::is_any_of(";"));
                    double x = boost::lexical_cast<double>(vec.at(0));
                    double y = boost::lexical_cast<double>(vec.at(1));
                    double z = boost::lexical_cast<double>(vec.at(2));

                    result.insert(
                        std::pair<std::string, glm::dvec3>(key, glm::dvec3(x, y, z)));
                }
            }
        } catch (std::exception &e) {
            logging::INFO(std::string("Failed to read filter parameter: ") +
            e.what());
        }

        element = element->NextSiblingElement("param");
    }

    return result;
}

Rotation XmlUtils::createRotationFromXml(tinyxml2::XMLElement* rotGroupNode){
    bool globalRotation = true;
    Rotation r = Rotation(glm::dvec3(1, 0, 0), 0);
    Rotation r2 = Rotation(glm::dvec3(1, 0, 0), 0);
    if (rotGroupNode == nullptr) {
        return r;
    }
    if (rotGroupNode->Attribute("rotations", "local"))
        globalRotation = false;

    tinyxml2::XMLElement *rotNodes = rotGroupNode->FirstChildElement("rot");
    while (rotNodes != nullptr) {
        std::string axis = rotNodes->Attribute("axis");
        double angle_rad = MathConverter::degreesToRadians(
            boost::lexical_cast<double>(rotNodes->Attribute("angle_deg")));

        if (angle_rad != 0) {
            if (axis == "z" || axis == "Z") {
                r2 = Rotation(glm::dvec3(0, 0, 1), angle_rad);
            }
            if (axis == "x" || axis == "X") {
                r2 = Rotation(glm::dvec3(1, 0, 0), angle_rad);
            }
            if (axis == "y" || axis == "Y") {
                r2 = Rotation(glm::dvec3(0, 1, 0), angle_rad);
            }
            if (globalRotation)
                r = r2.applyTo(r); // Global rotation
            else
                r = r.applyTo(r2); // Local rotation
        }

        rotNodes = rotNodes->NextSiblingElement("rot");
    }

    return r;
}

glm::dvec3 XmlUtils::createVec3dFromXml(
    tinyxml2::XMLElement* node,
    std::string attrPrefix
){
    if (node == nullptr) {
        throw HeliosException(
            "No node with attribute " + attrPrefix + "[xyz]"
        );
    }

    return glm::dvec3(
        boost::get<double>(
            getAttribute(node, attrPrefix + "x", "double", 0.0)
        ),
        boost::get<double>(
            getAttribute(node, attrPrefix + "y", "double", 0.0)
        ),
        boost::get<double>(
            getAttribute(node, attrPrefix + "z", "double", 0.0)
        )
    );
}

std::shared_ptr<NoiseSource<double>>
XmlUtils::createNoiseSource(tinyxml2::XMLElement *noise){
    std::shared_ptr<NoiseSource<double>> ns;

    // Instantiate considering type
    std::string type = noise->Attribute("type", "NORMAL");
    if (type == "UNIFORM") {
        double min = noise->DoubleAttribute("min", 0.0);
        double max = noise->DoubleAttribute("max", 1.0);
        ns = std::make_shared<UniformNoiseSource<double>>(
            UniformNoiseSource<double>(*DEFAULT_RG, min, max));
    } else {
        double mean = noise->DoubleAttribute("mean", 0.0);
        double stdev = noise->DoubleAttribute("stdev", 1.0);
        ns = std::make_shared<NormalNoiseSource<double>>(
            NormalNoiseSource<double>(*DEFAULT_RG, mean, stdev));
    }

    // Configure clipping
    bool clipEnabled = noise->BoolAttribute("clipEnabled", false);
    double clipMin = noise->DoubleAttribute("clipMin", 0.0);
    double clipMax = noise->DoubleAttribute("clipMax", 1.0);
    ns->setClipMin(clipMin).setClipMax(clipMax).setClipEnabled(clipEnabled);

    // Configure fixed behavior
    unsigned long fixedLifespan = noise->Unsigned64Attribute("fixedLifespan", 1L);
    ns->setFixedLifespan(fixedLifespan);

    return ns;
}

ObjectT XmlUtils::getAttribute(
    tinyxml2::XMLElement* element,
    std::string attrName,
    std::string type,
    ObjectT defaultVal,
    std::string const defaultMsg
){
    ObjectT result;
    try {
        if (!element->Attribute(attrName.c_str())) {
            throw HeliosException(
                "Attribute '" + attrName + "' does not exist!"
            );
        }
        std::string attrVal = element->Attribute(attrName.c_str());
        if (type == "bool") {
            if (attrVal == "1" || attrVal == "true")
                result = true;
            else if (attrVal == "0" || attrVal == "false")
                result = false;
            else {
                std::ostringstream s;
                s << "Attribute '" << attrName << "' does not exist!";
                logging::WARN(s.str());
                throw std::exception();
            }
        } else if (type == "int") {
            result = boost::lexical_cast<int>(attrVal);
        } else if (type == "float") {
            result = boost::lexical_cast<float>(attrVal);
        } else if (type == "double") {
            result = boost::lexical_cast<double>(attrVal);
        } else if (type == "string") {
            result = attrVal;
        } else {
            logging::ERR("ERROR: unknown type " + type);
        }
    }
    catch (std::exception &e) {
        std::stringstream ss;
        ss << "XML Assets Loader: Could not find attribute '" << attrName
           << "' of <" << element->Name() << "> element in line "
           << element->GetLineNum();
        logging::DEBUG(ss.str());
        ss.flush();
        ss.str("");

        if (!defaultVal.empty()) {
            result = defaultVal;
            ss << defaultMsg << " '" << attrName
               << "' : " << boost::apply_visitor(stringVisitor{}, defaultVal);
            logging::INFO(ss.str());
        }
        else {
            ss << "Exception:\n" << e.what() << "\n";
            ss << "ERROR: No default value specified for attribute '" << attrName
            << "'. Aborting.";
            logging::ERR(ss.str());
            exit(-1);
        }
    }

    return result;
}

bool XmlUtils::hasAttribute(
    tinyxml2::XMLElement *element,
    std::string attrName
){
    return element->Attribute(attrName.c_str()) != nullptr;
}

std::vector<std::shared_ptr<DynMotion>> XmlUtils::createDynMotionsVector(
        tinyxml2::XMLElement *element
){
    // Check there is at least one motion defined
    tinyxml2::XMLElement *xmlMotion = element->FirstChildElement("motion");
    if(xmlMotion == nullptr){
        throw HeliosException(
            "XmlUtils::createDynMotionsVector cannot create a vector from "
            "a XML element with not even a single motion child"
        );
    }
    std::vector<shared_ptr<DynMotion>> dms;
    RigidMotionR3Factory rm3f;
    tinyxml2::XMLAttribute const *motionAttr;

    // Handle children motion elements defining the sequence
    while(xmlMotion != nullptr){
        tinyxml2::XMLAttribute const *typeAttr = xmlMotion->FindAttribute(
            "type"
        );
        if(typeAttr == nullptr){ // Check type is given, it is a MUST
            throw HeliosException(
                "XmlUtils::createDynMotionsVector found a motion with no "
                "type. This is not allowed."
            );
        }
        std::string type = typeAttr->Value();
        // Handle motion self mode flag
        bool selfMode = false;
        tinyxml2::XMLAttribute const *selfModeAttr = xmlMotion->FindAttribute(
            "selfMode"
        );
        if(selfModeAttr != nullptr && selfModeAttr->BoolValue()){
            selfMode = true;
        }
        // Handle motion autoCRS flag
        double autoCRS = 0.0;
        tinyxml2::XMLAttribute const *autoCRSAttr = xmlMotion->FindAttribute(
            "autoCRS"
        );
        if(autoCRSAttr != nullptr) autoCRS = autoCRSAttr->DoubleValue();


        // Handle identity
        if(type=="identity"){
            dms.push_back(std::make_shared<DynMotion>(
                rm3f.makeIdentity(),
                selfMode
            ));
        }
        // Handle translation
        else if(type=="translation"){
            motionAttr = xmlMotion->FindAttribute("vec");
            if(motionAttr == nullptr){
                throw HeliosException(
                    "XmlUtils::createDynMotionsVector found a translation "
                    "motion with no vec attribute"
                );
            }
            arma::colvec vec(motionAttr->Value());
            dms.push_back(std::make_shared<DynMotion>(
                rm3f.makeTranslation(vec),
                selfMode,
                autoCRS
            ));
        }
        else if(type=="reflection"){
            motionAttr = xmlMotion->FindAttribute("ortho");
            if(motionAttr == nullptr){
                throw HeliosException(
                    "XmlUtils::createDynMotionsVector found a reflection "
                    "motion with no ortho attribute"
                );
            }
            arma::colvec ortho(motionAttr->Value());
            dms.push_back(std::make_shared<DynMotion>(
                rm3f.makeReflection(ortho),
                selfMode
            ));
        }
        else if(type=="glideplane"){
            motionAttr = xmlMotion->FindAttribute("ortho");
            if(motionAttr == nullptr){
                throw HeliosException(
                    "XmlUtils::createDynMotionsVector found a glideplane "
                    "motion with no ortho attribute"
                );
            }
            arma::colvec ortho(motionAttr->Value());
            motionAttr = xmlMotion->FindAttribute("shift");
            if(motionAttr == nullptr){
                throw HeliosException(
                    "XmlUtils::createDynMotionsVector found a glideplane "
                    "motion with no axis attribute"
                );
            }
            arma::colvec shift(motionAttr->Value());
            dms.push_back(std::make_shared<DynMotion>(
                rm3f.makeGlideReflection(ortho, shift),
                selfMode
            ));
        }
        else if(type=="rotation"){
            arma::colvec center({0, 0, 0});
            bool requiresCentering = false;
            motionAttr = xmlMotion->FindAttribute("center");
            if(motionAttr != nullptr){
                requiresCentering = true;
                center = arma::colvec(motionAttr->Value());
                dms.push_back(std::make_shared<DynMotion>(
                    rm3f.makeTranslation(-center),
                    selfMode,
                    autoCRS
                ));
            }
            motionAttr = xmlMotion->FindAttribute("axis");
            if(motionAttr == nullptr){
                throw HeliosException(
                    "XmlUtils::createDynMotionsVector found a rotation "
                    "motion with no axis attribute"
                );
            }
            arma::colvec axis(motionAttr->Value());
            motionAttr = xmlMotion->FindAttribute("angle");
            if(motionAttr == nullptr){
                throw HeliosException(
                    "XmlUtils::createDynMotionsVector found a rotation "
                    "motion with no angle attribute"
                );
            }
            double angle = MathConverter::degreesToRadians(
                motionAttr->DoubleValue()
            );
            dms.push_back(std::make_shared<DynMotion>(
                rm3f.makeRotation(axis, angle),
                selfMode
            ));
            if(requiresCentering){
                dms.push_back(std::make_shared<DynMotion>(
                    rm3f.makeTranslation(center),
                    selfMode,
                    -autoCRS  // Compensate the autoCRS of first translation
                ));
            }
        }
        else if(type=="helical"){
            motionAttr = xmlMotion->FindAttribute("axis");
            if(motionAttr == nullptr){
                throw HeliosException(
                    "XmlUtils::createDynMotionsVector found a helical "
                    "motion with no axis attribute"
                );
            }
            arma::colvec axis(motionAttr->Value());
            motionAttr = xmlMotion->FindAttribute("angle");
            if(motionAttr == nullptr){
                throw HeliosException(
                    "XmlUtils::createDynMotionsVector found a helical "
                    "motion with no angle attribute"
                );
            }
            double angle = MathConverter::degreesToRadians(
                motionAttr->DoubleValue()
            );
            motionAttr = xmlMotion->FindAttribute("glide");
            if(motionAttr == nullptr){
                throw HeliosException(
                    "XmlUtils::createDynMotionsVector found a helical "
                    "motion with no glide attribute"
                );
            }
            double glide = motionAttr->DoubleValue();
            dms.push_back(std::make_shared<DynMotion>(
                rm3f.makeHelical(axis, angle, glide),
                selfMode
            ));
        }
        else if(type=="rotsym"){
            motionAttr = xmlMotion->FindAttribute("axis");
            if(motionAttr == nullptr){
                throw HeliosException(
                    "XmlUtils::createDynMotionsVector found a rotsym "
                    "motion with no axis attribute"
                );
            }
            arma::colvec axis(motionAttr->Value());
            motionAttr = xmlMotion->FindAttribute("angle");
            if(motionAttr == nullptr){
                throw HeliosException(
                    "XmlUtils::createDynMotionsVector found a rotsym "
                    "motion with no angle attribute"
                );
            }
            double angle = MathConverter::degreesToRadians(
                motionAttr->DoubleValue()
            );
            motionAttr = xmlMotion->FindAttribute("center");
            if(motionAttr == nullptr){
                dms.push_back(std::make_shared<DynMotion>(
                    rm3f.makeRotationalSymmetry(axis, angle),
                    selfMode
                ));
            }
            else{
                arma::colvec center(motionAttr->Value());
                dms.push_back(std::make_shared<DynMotion>(
                    rm3f.makeRotationalSymmetry(axis, angle, center),
                    selfMode
                ));
            }
        }
        // Handle unexpected type
        else{
            std::stringstream ss;
            ss  << "XmlUtils::createDynMotionsVector found an unexpected "
                << "type \"" << type << "\"";
            throw HeliosException(ss.str());
        }

        // Next motion element, if any
        xmlMotion = xmlMotion->NextSiblingElement("motion");
    }

    return dms;
}

void XmlUtils::assertDocumentForAssetLoading(
    tinyxml2::XMLDocument &doc,
    std::string const &filename,
    std::string const &path,
    std::string const &type,
    std::string const &id,
    std::string const &caller
){
    if(doc.Error()){
        if(doc.ErrorID() == tinyxml2::XML_ERROR_FILE_NOT_FOUND){
            std::stringstream ss;
            ss    << "File \"" << filename << "\" was not found at:\n"
                  << "\"" << path << "\"\n"
                  << "Thus, it is not possible to load the asset \""
                  << type << "\":\"" << id << "\"";
            logging::ERR(ss.str());
        }
        else{
            std::stringstream ss;
            ss    << "It was not possible to load the asset \""
                  << type << "\":\"" << id << "\"\n"
                  << "At least not from file \"" << filename
                  << "\" at:\n" << "\"" << path << "\"";
            logging::ERR(ss.str());
        }
        std::stringstream ss;
        ss  << caller << " failed due to a document error";
        throw HeliosException(ss.str());
    }
}
