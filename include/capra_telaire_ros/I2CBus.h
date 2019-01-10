//
// Created by Alex on 25/11/18.
//

#ifndef CAPRA_CO2_DETECTION_I2CBUS_H
#define CAPRA_CO2_DETECTION_I2CBUS_H

#include <string>

class I2CBus {
public:
    //I2CBus();
    //~I2CBus();

    int getFile();

    bool openBus();

    void setBusName(std::string name);

private:
    int mFile;
    std::string mFilename;
};


#endif //CAPRA_CO2_DETECTION_I2CBUS_H
