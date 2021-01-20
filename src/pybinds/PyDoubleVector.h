#pragma once

#ifdef PYTHON_BINDING

#include <PyHeliosUtils.h>
#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for std::vector<double> class
 *
 * @see std::vector
 */
class PyDoubleVector{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    std::vector<double> *vec = nullptr;
    bool release = true;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyDoubleVector(std::vector<double> *vec) : vec(vec), release(false) {}
    PyDoubleVector(std::vector<double> const vec){
        this->vec = new std::vector<double>(vec);
        release = true;
    }
    virtual ~PyDoubleVector(){if(release && vec != nullptr) free(vec);}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    double get(long _index){
        size_t index = PyHeliosUtils::handlePythonIndex(_index, vec->size());
        return (*vec)[index];
    }
    void set(long _index, double value){
        size_t index = PyHeliosUtils::handlePythonIndex(_index, vec->size());
        (*vec)[index] = value;
    }
    void insert(double value){vec->push_back(value);}
    void erase(long _index){
        size_t index = PyHeliosUtils::handlePythonIndex(_index, vec->size());
        vec->erase(vec->begin() + index);
    }
    size_t length() {return vec->size();}
};

#endif