#pragma once

#include <string>
#include <stdexcept>
#include <surfaceinspector/util/Object.hpp>

using std::string;
using std::runtime_error;

namespace SurfaceInspector { namespace util{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Base class for surface inspector exceptions
 */
class SurfaceInspectorException : public runtime_error, Object {
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Surface inspector exception constructor
     * @param msg Surface inspector exception message
     */
    SurfaceInspectorException(std::string const msg = ""):runtime_error(msg){}
    virtual ~SurfaceInspectorException() = default;
};

}}