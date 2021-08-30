#pragma once

#include <rigidmotion/RigidMotion.h>

#include <memory>
#include <boost/serialization/void_cast.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/deque.hpp>

using namespace arma;
using rigidmotion::RigidMotion;

using std::shared_ptr;
using std::make_shared;

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Class which wraps the RigidMotion class to implement extra features
 *  such as the self mode control mechanism and computation of normal
 *  counterparts
 * @see rigidmotion::RigidMotion
 * @see DynMotion::selfMode
 * @see DynMotion::makeNormalCounterpart
 */
class DynMotion : public RigidMotion {
private:
    // ***  SERIALIZATION  *** //
    // *********************** //
    friend class boost::serialization::access;
    /**
     * @brief Serialize a dynamic motion to a stream of bytes
     * @tparam Archive Type of rendering
     * @param ar Specific rendering for the stream of bytes
     * @param version Version number for the dynamic motion
     */
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version){
        boost::serialization::void_cast_register<DynMotion, RigidMotion>();
        ar &boost::serialization::base_object<RigidMotion>(*this);
        ar &selfMode;
        ar &normalMode;
        //ar &C; // Not needed because it is in save/load construct
        //ar &A; // Not needed because it is in save/load construct
    }

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Specify if the self mode is enabled for this dynamic motion
     *  (true) or not (false).
     *
     * The self mode flag specifies that a dynamic motion should be applied to
     *  an object centered in the origin
     *
     * @see DynMotion::isSelfMode
     * @see DynMotion::setSelfMode
     * @see DynMotion::normalMode
     * @see DynMotionEngine::apply(DynMotion const &, mat const &, DynObject &)
     */
    bool selfMode = false;

    /**
     * @brief Specify if the normal mode is enabled for thid dynamic motion
     *  (true) or not (false).
     *
     * The normal mode specifies that a dynamic motion is mean to be applied
     *  to normals and not points. Generally speaking, enabling normal mode
     *  implies ignoring self mode for most cases, even if it is enabled.
     *
     * @see DynMotion::isNormalMode
     * @see DynMotion::setNormalMode
     * @see DynMotion::selfMode
     * @see DynMotionEngine::apply(DynMotion const &, mat const &, DynObject &)
     */
    bool normalMode = false;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Dynamic motion construction from a rigid motion as basis
     * @param rm Rigid motion as basis for the dynamic motion
     * @param selfMode Specify the initial value for self mode
     * @see rigidmotion::RigidMotion
     * @see DynMotion::selfMode
     */
    DynMotion(RigidMotion const & rm, bool selfMode=false) :
        RigidMotion(rm),
        selfMode(selfMode)
    {}
    /**
     * @brief Dynamic motion construction from given translation column vector
     *  and fixed origin transformation matrix
     * @param C Column vector representing the translation
     * @param A Matrix representing the fixed origin transformation
     * @see rigidmotion::RigidMotion::RigidMotion(colvec const, mat const)
     */
    DynMotion(colvec const &C, arma::mat const &A) : RigidMotion(C, A) {}
    virtual ~DynMotion() = default;

    // ***  NORMALS UTILS  *** //
    // *********************** //
    /**
     * @brief Check whether dynamic motion modifies normals when applied (true)
     *  or not (false).
     * @return True if applying the dynamic motion modifies normals, false
     *  otherwise
     */
    virtual bool checkModifiesNormal() const;
    /**
     * @brief Make the normal counterpart for the dynamic motion.
     *
     * It is, a version which preserves the fixed origin transformation but
     *  that does not change the position. It is mean to be used mainly to
     *  transform normals which must not change its position since they are
     *  normal unitary vectors defining direction only. Thus, when applying
     *  for instance a helical motion, the position of the dynamic object
     *  should be fully transformed but its normals must only be rotated with
     *  no transposition. Applying the normal counterpart of the helical motion
     *  with object normals guarantees this.
     *
     * More formally, let \f$f(X)\f$ be the affine application defining the
     *  base rigid motion at the core of the dynamic motion:
     *
     * \f[
     *  f(X) = C + AX
     * \f]
     *
     * Its normal counterpart namely \f$g(X)\f$ would be:
     *
     * \f[
     *  g(X) = \vec{0} + AX
     * \f]
     *
     * Also, dynamic motions which are normal counterparts have normal mode
     *  enabled by default and self mode inherited from source dynamic
     *  motion.
     *
     * @return Normal counterpart for the dynamic motion
     * @see rigidmotion::RigidMotion
     * @see DynMotion::makeNormalCounterpartPtr
     * @see DynMotion::normalMode
     */
    virtual DynMotion makeNormalCounterpart() const;
    /**
     * @brief Like makeNormalCounterpart method but returning a shared pointer
     *  instead of the raw object
     * @see DynMotion::makeNormalCounterpart
     */
    virtual inline shared_ptr<DynMotion> makeNormalCounterpartPtr() const
    {return make_shared<DynMotion>(makeNormalCounterpart());}

    // ***  GETTERs and SETTERs  *** //
    // ***************************** //
    /**
     * @brief Check if self mode is enabled for the dynamic motion (true) or
     *  not (false)
     * @return True if self mode is enabled, false otherwise
     * @see DynMotion::selfMode
     * @see DynMotion::setSelfMode
     */
    inline bool isSelfMode() const {return selfMode;}
    /**
     * @brief Enable or disable self mode
     * @param selfMode True to enable self mode, false to disable it
     * @see DynMotion::selfMode
     * @see DynMotion::isSelfMode
     */
    inline void setSelfMode(bool const selfMode) {this->selfMode = selfMode;}
    /**
     * @brief Check if normal mode is enabled for the dynamic motion (true) or
     *  not (false)
     * @return True if normal mode is enabled, false otherwise
     * @see DynMotion::normalMode
     * @see DynMotion::setNormalMode
     */
    inline bool isNormalMode() const {return normalMode;}
    /**
     * @brief Enable or disable normal mode
     * @param normalMode True to enable normal mode, false to disable it
     * @see DynMotion::normalMode
     * @see DynMotion::isNormalMode
     */
    inline void setNormalMode(bool const normalMode)
    {this->normalMode = normalMode;}
};
