#pragma once

#include <memory>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Functor container can be used to wrap any element so its functor ()
 *  is called while having a shared pointer to the contained element
 */
template <typename T>
class SmartSharedFunctorContainer{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Container object which must have a callable functor ()
     */
    std::shared_ptr<T> f;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for the smart shared functor container
     * @see SmartSharedFunctorContainer::f
     */
    SmartSharedFunctorContainer(std::shared_ptr<T> f) : f(f) {}
    virtual ~SmartSharedFunctorContainer() {}

    // ***  FUNCTOR OPERATOR  *** //
    // ************************** //
    /**
     * @brief Functor to the contained element's functor
     */
    virtual inline void operator() (){(*f)();}
};