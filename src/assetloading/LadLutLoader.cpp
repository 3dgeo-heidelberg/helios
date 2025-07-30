#include "LadLutLoader.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <logging.hpp>
#include <vector>

std::shared_ptr<LadLut>
LadLutLoader::load(std::string const& path, std::string const separator)
{
  // Open file input stream
  std::ifstream ifs;
  try {
    ifs = std::ifstream(path, std::ifstream::binary);
  } catch (std::exception& e) {
    std::stringstream ss;
    ss << "Failed to open ladlut file at \"" << path << "\"\n"
       << "EXCEPTION: " << e.what();
    logging::ERR(ss.str());
    exit(-1);
  }

  // Create LadLut
  std::shared_ptr<LadLut> ladlut = std::make_shared<LadLut>();

  // Parse
  std::string line;
  double x, y, z, g;
  while (getline(ifs, line)) {
    // Line to parts vector
    boost::algorithm::trim(line);
    std::vector<std::string> parts;
    boost::split(parts, line, boost::is_any_of(separator));

    // Extract values
    x = boost::lexical_cast<double>(parts[0]);
    y = boost::lexical_cast<double>(parts[1]);
    z = boost::lexical_cast<double>(parts[2]);
    g = boost::lexical_cast<double>(parts[3]);

    // Populate ladlut
    ladlut->X.push_back(x);
    ladlut->Y.push_back(y);
    ladlut->Z.push_back(z);
    ladlut->G.push_back(g);
  }

  // Prepare LadLut so it is ready to be used
  ladlut->computeAngles();

  // Return parsed LadLut
  return ladlut;
}
