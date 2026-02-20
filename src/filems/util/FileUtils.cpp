#include <FileUtils.h>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <logging.hpp>
namespace fs = boost::filesystem;

char const FileUtils::pathSeparator =
#ifdef _WIN32
  '\\'
#else
  '/'
#endif
  ;

void
resolveExtendedPath(std::string epath, std::vector<std::string>& paths)
{
  auto dir = fs::path(epath).remove_filename();
  if (fs::exists(dir)) {
    std::vector<std::string> files = FileUtils::getFilesByExpression(epath);
    paths.insert(paths.end(), files.begin(), files.end());
  }
}

std::vector<std::string>
FileUtils::handleFilePath(std::map<std::string, ObjectT>& params,
                          const std::vector<std::string>& assetsDir)
{
  std::string path;
  bool extendedFilePath = false;

  try {
    path = boost::get<std::string>(params["efilepath"]);
    extendedFilePath = true;
  } catch (std::exception& e) {
    try {
      path = boost::get<std::string>(params["filepath"]);
    } catch (std::exception& e2) {
      std::stringstream ss;
      ss << "No filepath was provided.\nEXCEPTION: " << e2.what();
      logging::ERR(ss.str());
    }
  }

  // Compile the resulting list of files
  std::vector<std::string> paths;
  if (extendedFilePath) {
    resolveExtendedPath(path, paths);
    for (auto assetPath : assetsDir)
      resolveExtendedPath((fs::path(assetPath) / path).string(), paths);
  } else {
    if (!fs::path(path).is_relative()) {
      paths.push_back(path);
    }
    for (auto assetPath : assetsDir) {
      if (fs::exists(fs::path(assetPath) / path)) {
        paths.push_back((fs::path(assetPath) / path).string());
        break;
      }
    }
  }

  return paths;
}

std::vector<std::string>
FileUtils::getFilesByExpression(std::string const pathExpression)
{
  std::vector<std::string> filePaths(0);
  fs::path path(pathExpression);
  fs::path parent(path);
  parent.remove_filename();
  fs::path fileExpression(path);
  boost::regex expr(fileExpression.filename().string());
  fs::directory_iterator diEnd;
  for (fs::directory_iterator di(parent); di != diEnd; ++di) {
    if (!fs::is_regular_file(di->status()))
      continue; // Skip non-files

    // Skip name which dont satisfy file name expression
    boost::smatch what;
    std::string filename = di->path().filename().string();
    if (!boost::regex_match(filename, what, expr))
      continue;

    // Register filePath because it matches regexp
    filePaths.push_back(di->path().string());
  }

  if (filePaths.empty()) {
    std::stringstream ss;
    ss << "Warning: No matching files were found "
          "in efilepath directory. "
          "Check regular expression.";
    logging::WARN(ss.str());
  }

  return filePaths;
}

void
FileUtils::extractExtensionAndPathWithoutExtension(std::string const& path,
                                                   std::string& ext,
                                                   std::string& pathNonExt)
{
  size_t const extPos = path.find_last_of(".");
  ext = path.substr(extPos);
  pathNonExt = path.substr(0, extPos);
}

std::string
FileUtils::craftPathWithSuffix(std::string const& pathNonExt,
                               std::string const& suffix,
                               std::string const& ext)
{
  return pathNonExt + suffix + ext;
}
