#include "capra_telaire_ros/Telaire6713.h"
#include <unistd.h>
#include <fcntl.h>
#include <algorithm>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <linux/i2c-dev.h>

#include "ros/ros.h"

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
    unsigned int writeBuff[5] = {0x04, 0x13, 0x8b, 0x00, 0x01};
    // ROS_INFO("PPM Message= %d", PPM);
    // std::copy(std::begin(PPM), std::end(PPM), std::begin(mWriteBuff));
    // ROS_INFO("mWriteBuff= %d", mWriteBuff);
    unsigned int readBuff[4];

    if(write(mI2CBus->getFile(), writeBuff, 5) != 5){
        usleep(50000);
        read(mI2CBus->getFile(), readBuff, 4);
        ROS_INFO("readBuff = {%d, %d, %d, %d}", readBuff[0], readBuff[1], readBuff[2], readBuff[3]);
        return readBuff[2] << 8 | readBuff[3];
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

bool Telaire6713::getBusAccess(){
    return (ioctl(mI2CBus->getFile(), I2C_SLAVE, mAddress) < 0);
}
