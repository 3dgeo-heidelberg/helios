#include <filems/write/core/HeliosWriter.h>
#include <util/HeliosException.h>

// ***  HELIOS WRITER METHODS  *** //
// ******************************* //
template <typename ... WriteArgs>
void HeliosWriter<WriteArgs ...>::finish(){
    // If there is no sync file writer to finish, throw an exception
    if(sfw == nullptr){
        throw HeliosException(
            "HeliosWriter::finish had not sync file writer (sfw) to "
            "finish"
        );
    }
    // Finish sync file writer, if any
    sfw->finish();
}
