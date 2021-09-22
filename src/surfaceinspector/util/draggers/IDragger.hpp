#pragma once

#include <surfaceinspector/util/Object.hpp>

using SurfaceInspector::util::Object;

namespace SurfaceInspector { namespace util { namespace draggers{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Dragger interface provide methods to drag elements from a given
 *  collection following a certain order. It is an interface, which means it
 *  cannot be directly instantiated. It must be implemented by concrete classes
 *  which implement its functions depending on the type of desired dragging
 * @tparam E Type of elements to drag
 * @tparam C Type of container to drag elements from
 */
template <typename E, typename C>
class IDragger : public Object{
public:
    // ***  INTERFACE  *** //
    // ******************* //
    /**
     * @brief Check if dragger supports dragging at least one more element
     * @return True if dragger supports dragging at leas one more element,
     *  false otherwise
     */
    virtual bool hasNext() = 0;
    /**
     * @brief Drag the next element from collection
     * @return Dragged element
     */
    virtual E next() = 0;
};
}}}