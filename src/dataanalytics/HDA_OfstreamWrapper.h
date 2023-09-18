#ifdef DATA_ANALYTICS

#include <vector>
#include <string>
#include <fstream>

// TODO Rethink : Document

namespace helios { namespace analytics{

class HDA_OfstreamWrapper{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    std::ofstream ofs;
    /**
     * @brief The separator between recorded elements
     */
    std::string sep;
    bool hasWrittenSomething;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    HDA_OfstreamWrapper(std::string const &outpath, std::string const &sep=",") :
        ofs(outpath, std::ios_base::out),
        sep(sep),
        hasWrittenSomething(false)
    {}
    virtual ~HDA_OfstreamWrapper() = default;

    // ***  OUTPUT FILE STREAM METHODS  *** //
    // ************************************ //
    inline bool is_open() {return ofs.is_open();}
    inline void close() {return ofs.close();}

    // ***  OUTPUT FILE STREAM OPERATORS  *** //
    // ************************************** //
    template <typename T>
    HDA_OfstreamWrapper & operator <<(T const &elem){
        if(hasWrittenSomething){
            ofs << sep << elem;
        }
        else{
            ofs << elem;
            hasWrittenSomething = true;
        }
        return *this;
    }

    HDA_OfstreamWrapper & operator <<(std::vector<double> const &elem){
        ofs << elem[0];
        for(size_t i = 1 ; i < elem.size(); ++i){
            ofs << sep << elem[i];
        }
        ofs << "\n";
        return *this;
    }

};

}}
#endif