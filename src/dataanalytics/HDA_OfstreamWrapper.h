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
    /**
     * @brief The output file stream used to write to a file.
     */
    std::ofstream ofs;
    /**
     * @brief The separator between recorded elements.
     */
    std::string sep;
    /**
     * @brief Boolean flag to control whether the HDA_OfstreamWrapper has
     *  written something or not. Once something has been written, the flag
     *  will be set to true. By default, it is initialized to false.
     */
    bool hasWrittenSomething;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Build a HDA_OfstreamWrapper to handle writing records to files.
     * @param outpath The path to the output file.
     * @see HDA_OfstreamWrapper::sep
     */
    HDA_OfstreamWrapper(
        std::string const &outpath, std::string const &sep=","
    ) :
        ofs(outpath, std::ios_base::out),
        sep(sep),
        hasWrittenSomething(false)
    {}
    virtual ~HDA_OfstreamWrapper() = default;

    // ***  OUTPUT FILE STREAM METHODS  *** //
    // ************************************ //
    /**
     * @brief Wraps the is_open method of the output file stream class.
     */
    inline bool is_open() {return ofs.is_open();}
    /**
     * @brief Wraps the close method of the output file stream class.
     */
    inline void close() {return ofs.close();}

    // ***  OUTPUT FILE STREAM OPERATORS  *** //
    // ************************************** //
    /**
     * @brief Default stream input operator, i.e., <<.
     * @tparam T The type of element to be written.
     * @param elem The element to be written.
     * @return A reference to the HDA_OfstreamWrapper itself for fluent
     *  programming purposes.
     */
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

    /**
     * @brief Stream input operator to handle vectors of doubles as element.
     * @param elem The vector of doubles as element.
     * @return A reference to the HDA_OfstreamWrapper itself for fluent
     *  programming purposes.
     */
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