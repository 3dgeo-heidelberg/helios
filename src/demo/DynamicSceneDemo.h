#ifdef PCL_BINDING
#pragma once

namespace HeliosDemos{

using visualhelios::VHDynObjectXYZAdapter;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Dynamic scene demo
 *
 * This demo implements the rendering of a given dynamic scene
 */
class DynamicSceneDemo : public BaseDemo{
    // TODO Rethink : To be implemented yet
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Dynamic scene demo constructor
     */
    DynamicSceneDemo() : BaseDemo("Dynamic scene demo") {}
    virtual ~DynamicSceneDemo() = default;

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseDemo::run
     */
    void run() override;
};

}

#endif