#pragma once

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Interface defining the behaviors that must be supported by any
 *  object that can notify to a basic dynamic grove
 * @see BasicDynGrove
 */
class BasicDynGroveSubject{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    virtual ~BasicDynGroveSubject() = default;

    // ***  BASIC DYNGROVE SUBJECT METHODS  *** //
    // **************************************** //
    /**
     * @brief Set the subject identifier to be given one
     * @param id New identifier for the subject
     */
    virtual void setGroveSubjectId(std::size_t const id) = 0;
    /**
     * @brief Get the subject identifier
     * @return Subject identifier
     */
    virtual std::size_t getGroveSubjectId() = 0;
};