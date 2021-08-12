#include <scene/dynamic/DynObject.h>

// ***  U T I L  *** //
// ***************** //
size_t DynObject::countVertices(){
    size_t m = 0;
    Primitive * primitive;
    for(size_t i = 0 ; i < primitives.size(); ++i){
        primitive = primitives[i];
        m += primitive->getNumVertices();
    }
    return m;
}

arma::mat DynObject::matrixFromPrimitives(
    std::function<arma::colvec(Vertex const *)> get
){
    return matrixFromPrimitives(countVertices(), get);
}
arma::mat DynObject::matrixFromPrimitives(
    size_t const m,
    std::function<arma::colvec(Vertex const *)> get
){
    arma::mat X(3, m);
    size_t i = 0;
    for(size_t j = 0 ; j < primitives.size() ; ++j){
        Primitive * primitive = primitives[j];
        Vertex const * vertices = primitive->getVertices();
        for(size_t k = 0 ; k < primitive->getNumVertices() ; ++k, ++i){
            X.col(i) = get(vertices + k);
        }
    }
    return X;
}

void DynObject::matrixToPrimitives(
    std::function<void(Vertex *, arma::colvec const&)> set,
    arma::mat const &X
){
    matrixToPrimitives(countVertices(), set, X);
}
void DynObject::matrixToPrimitives(
    size_t const m,
    std::function<void(Vertex *, arma::colvec const&)> set,
    arma::mat const &X
){
    size_t i = 0;
    for(size_t j = 0 ; j < primitives.size() ; ++j){
        Primitive * primitive = primitives[j];
        Vertex *vertices = primitive->getVertices();
        for(size_t k = 0 ; k < primitive->getNumVertices() ; ++k, ++i){
            set(vertices + k, X.col(i));
        }
    }
}

arma::mat DynObject::positionMatrixFromPrimitives(){
    return positionMatrixFromPrimitives(countVertices());
}
arma::mat DynObject::positionMatrixFromPrimitives(size_t const m){
    return matrixFromPrimitives(m, [](Vertex const *p) -> arma::colvec{
        arma::colvec x(3);
        x(0) = p->getX();
        x(1) = p->getY();
        x(2) = p->getZ();
        return x;
    });
}

arma::mat DynObject::normalMatrixFromPrimitives(){
    return normalMatrixFromPrimitives(countVertices());
}
arma::mat DynObject::normalMatrixFromPrimitives(size_t const m){
    return matrixFromPrimitives(m, [](Vertex const *p) -> arma::colvec{
        arma::colvec x(3);
        x(0) = p->normal.x;
        x(1) = p->normal.y;
        x(2) = p->normal.z;
        return x;
    });
}

void DynObject::updatePrimitivesPositionFromMatrix(arma::mat const &X){
    updatePrimitivesPositionFromMatrix(countVertices(), X);
}
void DynObject::updatePrimitivesPositionFromMatrix(
    size_t const m,
    arma::mat const &X
){
    matrixToPrimitives(
        m,
        [](Vertex *p, arma::colvec const &x) -> void{
            p->pos.x = x(0);
            p->pos.y = x(1);
            p->pos.z = x(2);
        },
        X
    );
}

void DynObject::updatePrimitivesNormalFromMatrix(arma::mat const &X){
    updatePrimitivesNormalFromMatrix(countVertices(), X);
}
void DynObject::updatePrimitivesNormalFromMatrix(
    size_t const m,
    arma::mat const &X
){
    matrixToPrimitives(
        m,
        [](Vertex *p, arma::colvec const &x) -> void{
            p->normal.x = x(0);
            p->normal.y = x(1);
            p->normal.z = x(2);
        },
        X
    );
}

