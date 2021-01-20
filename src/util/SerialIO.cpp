#include <SerialIO.h>

// ***  SINGLETON  *** //
// ******************* //
SerialIO * SerialIO::instance = nullptr;
SerialIO * SerialIO::getInstance(){
    if(instance == nullptr){
        instance = new SerialIO();
    }
    return instance;
}

