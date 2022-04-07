#pragma once

#include <assetloading/EggAsset.h>
#include <platform/InterpolatedMovingPlatform.h>
#include <sim/comps/SimulationStepLoop.h>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Egg class that can hatch to a full InterpolatedMovingPlatform class
 *
 * It is mainly used to facilitate asset loading of InterpolatedMovingPlatform
 * @see InterpolatedMovingPlatform
 */
class InterpolatedMovingPlatformEgg :
    public EggAsset<
        InterpolatedMovingPlatform,
        SimulationStepLoop &
    >,
    public Platform
{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    InterpolatedMovingPlatform::InterpolationScope scope;
    std::shared_ptr<TemporalDesignMatrix<double, double>> tdm;
    std::shared_ptr<DiffDesignMatrix<double, double>> ddm;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    InterpolatedMovingPlatformEgg() :
        EggAsset(),
        tdm(nullptr),
        ddm(nullptr)
    {}
    virtual ~InterpolatedMovingPlatformEgg() = default;

    // ***  EGG METHODS  *** //
    // ********************* //
    InterpolatedMovingPlatform hatch(SimulationStepLoop &stepLoop) override{
        return InterpolatedMovingPlatform(
            stepLoop,
            *tdm,
            *ddm,
            scope
        );
    }
    std::shared_ptr<InterpolatedMovingPlatform> smartHatch(
        SimulationStepLoop &stepLoop
    ) override{
        return std::make_shared<InterpolatedMovingPlatform>(
            stepLoop,
            *tdm,
            *ddm,
            scope
        );
    }
};