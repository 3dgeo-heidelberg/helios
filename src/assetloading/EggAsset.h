#pragma once

#include <assetloading/Asset.h>

#include <memory>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief An EggAsset is an asset that can hatch to its full class
 * @tparam FullClass The class that the egg can hatch too
 * @tparam HatchArgs The arguments that must be provided for the hatch
 *  operation to be feasible
 * @see Asset
 */
template <typename FullClass, typename ... HatchArgs>
class EggAsset : public Asset{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for EggAsset
     */
    EggAsset() : Asset() {}
    virtual ~EggAsset() = default;

    // ***  EGG METHODS  *** //
    // ********************* //
    /**
     * @brief Hatch the egg class so the full class arises
     * @return Full class from the egg class
     */
    virtual FullClass hatch(HatchArgs ...) = 0;
    /**
     * @brief Like the hatch method but returning the full class as a shared
     *  smart pointer
     * @see EggAsset::hatch
     */
    virtual std::shared_ptr<FullClass> smartHatch(HatchArgs ...) = 0;

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @see Asset::isEgg
     */
    bool isEgg() const override {return true;}


};
