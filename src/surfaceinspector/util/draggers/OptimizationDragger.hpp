#pragma once

#include <surfaceinspector/util/SurfaceInspectorException.hpp>
#include <surfaceinspector/util/draggers/IDragger.hpp>

#include <vector>

using SurfaceInspector::util::SurfaceInspectorException;
using SurfaceInspector::util::draggers::IDragger;

using std::vector;

namespace SurfaceInspector { namespace util { namespace draggers{
/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Optimization dragger interface provide methods expanding dragger
 *  definition to become a optimization dragger which operates over a vector
 *  data structure.
 * @tparam E Type of elements to drag
 * @see SurfaceInspector::util::draggers::IDragger
 */
template <typename E>
class OptimizationDragger : public IDragger<E, vector<E>>{
protected:
    // ***  ABSTRACT METHODS  *** //
    // ************************** //
    /**
     * @brief Pick the element that must be returned when calling next for
     *  current dragger status.
     *  It defines the optimization picking criteria itself.
     * @return Element that must be returned when calling next for current
     *  dragger status.
     */
    virtual E pick() = 0;
    /**
     * @brief Update the dragger status advancing one step
     */
    virtual void update() = 0;

public:
    // ***  CONCRETE METHODS  *** //
    // ************************** //
    /**
     * @brief Implementation of next method for optimization draggers.
     *
     * First update the dragger status, then return picked element
     *
     * @return Dragged element
     * @see SurfaceInspector::util::draggers::IDragger::hasNext
     * @see SurfaceInspector::util::draggers::IDragger::next
     * @see OptimizationDragger::update
     * @see OptimizationDragger::pick
     */
    E next() override {
        // Check there are more elements to drag
        if(!this->hasNext()){
            throw SurfaceInspectorException(
                "Optimization dragger has no more elements to drag"
            );
        }

        // Pick, update and return
        update();
        return pick();
    }
};
}}}