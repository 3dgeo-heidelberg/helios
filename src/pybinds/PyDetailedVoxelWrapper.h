#pragma once

#include <PyPrimitiveWrapper.h>

namespace pyhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for DetailedVoxel class
 *
 * @see DetailedVoxel
 */
class PyDetailedVoxelWrapper : public PyPrimitiveWrapper{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyDetailedVoxelWrapper(DetailedVoxel *dv) : PyPrimitiveWrapper(dv) {}
    virtual ~PyDetailedVoxelWrapper() = default;

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    int getNbEchos() {return ((DetailedVoxel *)prim)->getNbEchos(); }
    void setNbEchos(int nbEchos)
    {((DetailedVoxel *)prim)->setNbEchos(nbEchos); }
    int getNbSampling() {return ((DetailedVoxel *)prim)->getNbSampling(); }
    void setNbSampling(int nbSampling)
    {((DetailedVoxel *)prim)->setNbSampling(nbSampling); }
    size_t getNumberOfDoubleValues()
    {return ((DetailedVoxel *)prim)->getNumberOfDoubleValues();}
    double getDoubleValue(size_t index)
    {return ((DetailedVoxel *)prim)->getDoubleValue(index);}
    void setDoubleValue(size_t index, double value)
    {((DetailedVoxel *)prim)->setDoubleValue(index, value);}
    double getMaxPad() {return ((DetailedVoxel *)prim)->getMaxPad();}
    void setMaxPad(double maxPad) {((DetailedVoxel *)prim)->setMaxPad(maxPad);}
};

}
