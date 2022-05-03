#pragma once

namespace helios { namespace filems {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Abstract class defining the fundamentals of any file reading strategy
 * @tparam ReadType Type of what is read from file
 */
template <typename ReadType>
class ReadingStrategy{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Default constructor for reading strategy
     */
    ReadingStrategy() = default;
    virtual ~ReadingStrategy() = default;

    // ***  READING STRATEGY METHODS  *** //
    // ********************************** //
    /**
     * @brief Read from file
     * @return What has been read from file
     */
    virtual ReadType read() = 0;
};

}}