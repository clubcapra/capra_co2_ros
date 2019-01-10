//
// Created by alex on 25/11/18.
//

#ifndef CAPRA_CO2_DETECTION_TELAIRE6713_H
#define CAPRA_CO2_DETECTION_TELAIRE6713_H

#include "I2CBus.h"

class Telaire6713 {

    const unsigned int Status[5] = { 0x04, 0x13, 0x8a, 0x00, 0x01 };
    const unsigned int PPM[5] = { 0x04, 0x13, 0x8b, 0x00, 0x01 };
    const unsigned int StatusABC[5] = { 0x04, 0x13, 0x8a, 0x00, 0x01 };

public:
    //Telaire6713();
    //~Telaire6713();

    int getStatus();
    int getPPM();
    int checkABC();
    //int calibrate();
    void setAddress(int a);
    void setI2CBus(I2CBus * i2CBus);

private:
    I2CBus * mI2CBus;
    unsigned int mReadBuff[4] = {0};
    unsigned int mWriteBuff[5] = {0};
    int mAddress{0x15};

    //Functions
    bool mWrite();
};

#endif //CAPRA_CO2_DETECTION_TELAIRE6713_H
