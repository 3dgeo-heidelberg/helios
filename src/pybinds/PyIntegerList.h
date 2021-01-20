#pragma once

#ifdef PYTHON_BINDING

#include <PyHeliosUtils.h>
#include <list>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for std::list<int> class
 *
 * @see std::list
 */
class PyIntegerList{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    std::list<int> &list;

    // ***  CONSTRUCTION  *** //
    // ********************** //
    PyIntegerList(std::list<int> &list) : list(list) {}
    virtual ~PyIntegerList() {}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    int get(long _index){
        size_t index = PyHeliosUtils::handlePythonIndex(_index, list.size());
        std::list<int>::iterator it = list.begin();
        for(size_t i = 0 ; i < index ; i++)it++;
        return *it;
    }
    void set(long _index, int value){
        size_t index = PyHeliosUtils::handlePythonIndex(_index, list.size());
        std::list<int>::iterator it = list.begin();
        for(size_t i = 0 ; i < index ; i++)it++;
        *it = value;
    }
    void insert(long _index, int value){
        size_t index = PyHeliosUtils::handlePythonIndex(_index, list.size());
        std::list<int>::iterator it = list.begin();
        for(size_t i = 0 ; i < index ; i++)it++;
        list.insert(it, value);
    }
    void erase(long _index){
        size_t index = PyHeliosUtils::handlePythonIndex(_index, list.size());
        std::list<int>::iterator it = list.begin();
        for(size_t i = 0 ; i < index ; i++)it++;
        list.erase(it);
    }
    size_t length() {return list.size();}
};

#endif