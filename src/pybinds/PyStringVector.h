#pragma once

#include <PyHeliosUtils.h>
#include <vector>
#include <string>

namespace pyhelios{


class PyStringVector{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    std::vector<std::string> *vec = nullptr;
    bool release = true;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyStringVector(std::vector<std::string> *vec) : vec(vec), release(false) {}
    PyStringVector(std::vector<std::string> const vec){
        this->vec = new std::vector<std::string>(vec);
        release = true;
    }
    virtual ~PyStringVector(){if(release && vec != nullptr) free(vec);}

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    std::string get(long _index){
        size_t index = PyHeliosUtils::handlePythonIndex(_index, vec->size());
        return (*vec)[index];
    }
    void set(long _index, std::string value){
        size_t index = PyHeliosUtils::handlePythonIndex(_index, vec->size());
        (*vec)[index] = value;
    }
    void insert(std::string value){vec->push_back(value);}
    void erase(long _index){
        size_t index = PyHeliosUtils::handlePythonIndex(_index, vec->size());
        vec->erase(vec->begin() + index);
    }
    size_t length() {return vec->size();}
};

}
