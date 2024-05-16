#pragma once

#ifdef PYTHON_BINDING

#include <glm/glm.hpp>
#include <memory>
#include <armadillo>

namespace pyhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Wrapper to communicate glm::dvec3 with python
 */
class PythonDVec3 {
    // ***  ATTRIBUTES  *** //
    // ******************** //
private:
    bool release = 1;

public:
    glm::dvec3 * v = nullptr;

    // ***  CONSTRUCTION  *** //
    // ********************** //
    PythonDVec3(glm::dvec3 const v) {
        this->v = new glm::dvec3(v);
        release = true;
    }
    PythonDVec3(glm::dvec3 *v){
        this->v = v;
        release = false;
    }
    PythonDVec3(arma::colvec const v){
        this->v = new glm::dvec3(v[0], v[1], v[2]);
        release = true;
    }
    virtual ~PythonDVec3(){
        if(release && v!=nullptr) delete v;
    }

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    double getX() {return v->x;}
    void setX(double x) {v->x = x;}
    double getY() {return v->y;}
    void setY(double y) {v->y = y;}
    double getZ() {return v->z;}
    void setZ(double z) {v->z = z;}

};

}

#endif