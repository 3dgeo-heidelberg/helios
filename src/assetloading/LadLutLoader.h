#pragma once

#include <LadLut.h>
#include <string>
#include <memory>

/**
 * @brief Loader for Leaf Angle Distribution Look Up Tables
 */
class LadLutLoader{
public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Build a new loader for leaf angle distribution look up tables
     */
    LadLutLoader() = default;
    virtual ~LadLutLoader() = default;

    // ***  L O A D  *** //
    // ***************** //
    /**
     * @brief Load LadLut from specified file
     * @param path Path to file containing LadLut data
     * @param separator Separator between data records
     * @return Loaded LadLut
     */
    std::shared_ptr<LadLut> load(
        std::string const &path,
        std::string const separator = ","
    );
};