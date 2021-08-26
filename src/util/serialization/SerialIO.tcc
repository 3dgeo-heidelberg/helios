#pragma once

#include <iostream>
#include <fstream>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <logging.hpp>

// *** SERIAL IO METHODS *** //
// ************************* //
template<class SerialClass>
void SerialIO::write(
    std::string const &path,
    SerialClass const *object,
    bool const fastCompression
){
    std::ofstream ofs;
    int compressionMode = (fastCompression) ?
        boost::iostreams::zlib::best_speed :  // Reduce exec. time at most
        boost::iostreams::zlib::best_compression;  // Reduce size at most

    try{
        ofs.open(path, std::ios_base::out | std::ios_base::binary);
        boost::iostreams::filtering_ostream compressedOut;
        boost::iostreams::zlib_params zp(compressionMode);
        compressedOut.push(boost::iostreams::zlib_compressor(zp));
        compressedOut.push(ofs);
        boost::archive::binary_oarchive oa(compressedOut);
        oa & object;
    }
    catch(std::exception& e){
        logging::WARN(e.what());
    }

    ofs.close();
}

template<typename SerialClass>
SerialClass * SerialIO::read(
    std::string const& path,
    bool const fastCompression
){
    SerialClass *object = NULL;
    std::ifstream ifs;
    int compressionMode = (fastCompression) ?
        boost::iostreams::zlib::best_speed :  // Reduce size at most
        boost::iostreams::zlib::best_compression;  // Reduce exec. time at most

    try {
        ifs.open(path, std::ios_base::in | std::ios_base::binary);
        boost::iostreams::filtering_istream compressedIn;
        boost::iostreams::zlib_params zp(compressionMode);
        compressedIn.push(boost::iostreams::zlib_decompressor(zp));
        compressedIn.push(ifs);
        boost::archive::binary_iarchive ia(compressedIn);
        ia & object;
    }
    catch(std::exception &e){
        std::stringstream ss;
        ss << "SerialIO::read EXCEPTION:\n\t" << e.what();
        logging::WARN(ss.str());
    }

    ifs.close();
    return object;
}
