#pragma once


#include <string>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Utils for Input/Output operations of serializable objects.
 */
class SerialIO {
private:
    // ***  SINGLETON  *** //
    // ******************* //
    /**
     * @brief Pointer to singleton instance of SerialIO
     */
    static SerialIO * instance;
    /**
     * @brief Singleton constructor
     */
    SerialIO() = default;
public:
    // ***  SINGLETON  *** //
    // ******************* //
    /**
     * Obtain the singleton instance of SerialIO.
     * It will be instantiated iff no instance is available
     * @return Singleton instance of SerialIO
     */
    static SerialIO * getInstance();

    virtual ~SerialIO() = default;

    // *** SERIAL IO METHODS *** //
    // ************************* //
    /**
     * Serialize an object and write it to a file.
     * @tparam SerialClass Specify the serializable type to use
     * @param path Path to write the serialized object to
     * @param object Object to be serialized and stored
     * @param fastCompression Specify if use the fast compression mode
     *  (true), which means the compression will be as fast as possible,
     *  or the best compression mode (false), which means the compression
     *  will lead to a size as small as possible at expenses of execution time.
     */
    template<class SerialClass>
    void write(
        std::string const& path,
        SerialClass const *object,
        bool fastCompression=true
    );

    /**
     * Read a serialized object and build corresponding instance from it.
     * @tparam SerialClass Specify the serializable type to use
     * @param path Path to read the serialized object from
     * @param fastCompression Specify if use the fast compression mode
     *  (true), which means the compression will be as fast as possible,
     *  or the best compression mode (false), which means the compression
     *  will lead to a size as small as possible at expenses of execution time.
     * @return Instance built from serialized object file
     */
    template<typename SerialClass>
    SerialClass * read(
        std::string const& path,
        bool const fastCompression=true
    );
};

#include <SerialIO.tcc>