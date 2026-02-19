#pragma once

#include <boost/iostreams/filtering_stream.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>

namespace helios {
namespace filems {

inline void
writeZippedStringRecord(boost::iostreams::filtering_ostream& out,
                        std::string const& record)
{
  std::uint64_t const size = record.size();
  out.write(reinterpret_cast<char const*>(&size),
            static_cast<std::streamsize>(sizeof(size)));
  if (size > 0) {
    out.write(record.data(), static_cast<std::streamsize>(size));
  }
}

inline bool
readZippedStringRecord(boost::iostreams::filtering_istream& in,
                       std::string& record)
{
  std::uint64_t size = 0;
  in.read(reinterpret_cast<char*>(&size),
          static_cast<std::streamsize>(sizeof(size)));
  if (!in) {
    if (in.eof()) {
      return false;
    }
    throw std::runtime_error("Failed to read record size from zipped stream");
  }

  record.resize(size);
  if (size > 0) {
    in.read(record.data(), static_cast<std::streamsize>(size));
    if (!in) {
      throw std::runtime_error(
        "Failed to read record payload from zipped stream");
    }
  }
  return true;
}

}
}
