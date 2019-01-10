#include "capra_telaire_ros/I2CBus.h"
#include <fcntl.h>

int I2CBus::getFile() {
    return mFile;
}

void I2CBus::setBusName(std::string name) {
    mFilename.assign(name);
}


bool I2CBus::openBus() {
    return ((mFile = open(mFilename.c_str(), O_RDWR)) > 0);
}
