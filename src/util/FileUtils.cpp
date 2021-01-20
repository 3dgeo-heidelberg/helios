#include <FileUtils.h>
#include <logging.hpp>
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
namespace fs = boost::filesystem;

std::vector<std::string> FileUtils::getFilesByExpression(
    std::string const pathExpression
){
    std::vector<std::string> filePaths(0);
    fs::path path(pathExpression);
    fs::path parent(path);
    parent.remove_filename();
    fs::path fileExpression(path);
    boost::regex expr(fileExpression.filename().string());
    fs::directory_iterator diEnd;
    for(fs::directory_iterator di(parent) ; di != diEnd ; ++di){
        if(!fs::is_regular_file(di->status())) continue; // Skip non-files

        // Skip name which dont satisfy file name expression
        boost::smatch what;
        std::string filename = di->path().filename().string();
        if(!boost::regex_match(filename, what, expr)) continue;

        // Register filePath because it matches regexp
        filePaths.push_back(di->path().string());
    }
    return filePaths;
}

void FileUtils::unzipFile(
    std::string const inputPath,
    std::string const outputPath
){
    std::string str = "START";
    std::ifstream ifs;
    std::ofstream ofs;
    int compressionMode = // Must match the one used by ZipSyncFileWriter
        boost::iostreams::zlib::best_compression;

    try{
        // Prepare input stream
        ifs.open(inputPath, std::ios_base::in | std::ios_base::binary);
        boost::iostreams::filtering_istream compressedIn;
        boost::iostreams::zlib_params zp(compressionMode);
        compressedIn.push(boost::iostreams::zlib_decompressor(zp));
        compressedIn.push(ifs);
        boost::archive::binary_iarchive ia(compressedIn);

        // Prepare output stream
        ofs.open(outputPath, std::ios_base::out);

        // Decompress
        while(str.length() > 0) {
            ia >> str;
            ofs << str;
        }
    }
    catch(std::exception &e){
        std::stringstream ss;
        ss << "FileUtils::unzipFile EXCEPTION:\n\t" << e.what();
        logging::WARN(ss.str());
    }

}
