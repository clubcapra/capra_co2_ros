#include "capra_telaire_ros/Telaire6713.h"
#include <unistd.h>
#include <fcntl.h>
#include <algorithm>

bool Telaire6713::mWrite() {
    return write(mI2CBus->getFile(), mWriteBuff, 5) != 5;
}

int Telaire6713::getStatus() {
    //mWriteBuff = Status;//{0x04, 0x13, 0x8a, 0x00, 0x01};
    std::copy(std::begin(Status), std::end(Status), std::begin(mWriteBuff)); 
    if(mWrite()){
        usleep(100000);
        read(mI2CBus->getFile(), mReadBuff, 4);
        return mReadBuff[2] << 8 | mReadBuff[3];
    }
}

int Telaire6713::getPPM() {
    //mWriteBuff = PPM;//{0x04, 0x13, 0x8b, 0x00, 0x01};
    std::copy(std::begin(PPM), std::end(PPM), std::begin(mWriteBuff));
    if(mWrite()){
        usleep(100000);
        read(mI2CBus->getFile(), mReadBuff, 4);
        return mReadBuff[2] << 8 | mReadBuff[3];
    }
}

int Telaire6713::checkABC() {
    //mWriteBuff = StatusABC;//{0x04, 0x13, 0x8a, 0x00, 0x01};
    std::copy(std::begin(StatusABC), std::end(StatusABC), std::begin(mWriteBuff));
    if(mWrite())
    {
        usleep(100000);
        read(mI2CBus->getFile(), mReadBuff, 4);
        return mReadBuff[2] << 8 | mReadBuff[3];
    }
}

void Telaire6713::setAddress(int a) {
    mAddress = a;
}

void Telaire6713::setI2CBus(std::shared_ptr<I2CBus>& i2CBus) {
    mI2CBus = i2CBus;
}
