#include "ros/ros.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "telaire");

    ros::NodeHandle n;

    I2CBus * i2CBus{ new I2CBus };
    i2CBus->setBusName("/dev/i2c-1");
    i2CBus->openBus();

    Telaire6713 * telaire6713{ new Telaire6713 };
    //telaire6713->setAddress(0x15);
    telaire6713->setI2CBus(i2CBus);

    while (ros::ok()){
        std::cout << "PPM Reading: " << telaire6713->getPPM() << std::endl;
        std::cout << "********" << std::endl;
        usleep(500000);
    }

    return EXIT_SUCCESS;
}
