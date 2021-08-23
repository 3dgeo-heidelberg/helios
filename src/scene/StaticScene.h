#pragma once

#include <vector>
#include <memory>


#include <scene/Scene.h>
#include <assetloading/ScenePart.h>

using std::vector;
using std::shared_ptr;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Static scene basic implementation
 *
 * A static scene is a simple scene which is aware of primitives composing
 *  each object in the scene.
 *
 * @see DynScene
 */
class StaticScene : public Scene{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Static objects composing the scene
     */
    vector<shared_ptr<ScenePart>> staticObjs;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Static scene default constructor
     */
    StaticScene() = default;
    ~StaticScene() override {}
    /**
     * @brief Copy constructor for static scene
     * @param ss Static scene to be copied
     */
    StaticScene(StaticScene &ss);
    /**
     * @brief Build a static scene using given scene as basis
     * @param s Basis scene for static scene
     */
    StaticScene(Scene &s) : Scene(s) {}

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Append given static object to the scene
     * @param obj Static object to be appended to the scene
     * @see ScenePart
     */
    inline void appendStaticObject(shared_ptr<ScenePart> obj)
    {staticObjs.push_back(obj);}
    /**
     * @brief Obtain static object at given index
     * @param index Index of static object to be obtained
     * @return Static object at given index
     */
    inline shared_ptr<ScenePart> getStaticObject(size_t const index)
    {return staticObjs[index];}
    /**
     * @brief Set static object at given index
     * @param index Index of static object to be setted
     * @param obj New static object
     */
    inline void setStaticObject(size_t const index, shared_ptr<ScenePart> obj)
    {staticObjs[index] = obj;}
    /**
     * @brief Remove static object at given index
     * @param index Index of static object to be removed
     */
    inline void removeStaticObject(size_t const index)
    {staticObjs.erase(staticObjs.begin()+index);}
    /**
     * @brief Remove all static objects from the static scene
     */
    inline void clearStaticObjects(){staticObjs.clear();}
    /**
     * @brief Obtain the number of static objects in the scene
     * @return Number of static objects in the scene
     */
    inline size_t numStaticObjects() {return staticObjs.size();}
};