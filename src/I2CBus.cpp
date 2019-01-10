//
// Created by alex on 25/11/18.
//

#include "I2CBus.h"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
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
