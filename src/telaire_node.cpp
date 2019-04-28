#include "ros/ros.h"
#include "std_msgs/String.h"
#include "capra_telaire_ros/I2CBus.h"
#include "capra_telaire_ros/Telaire6713.h"

#include <string>
#include <memory>

int main(int argc, char **argv) {

    ros::init(argc, argv, "telaire");

    ros::NodeHandle n;
    ros::Publisher ppm_publisher = n.advertise<std_msgs::String>("ppm", 1);

    std::shared_ptr<I2CBus> i2CBus(new I2CBus);
    i2CBus->setBusName("/dev/i2c-1");
    if(i2CBus->openBus()){
        ROS_INFO("Failed to open i2c bus");
    }

    Telaire6713 telaire;
    telaire.setI2CBus(i2CBus);

    if (telaire.getBusAccess())
    {
        ROS_INFO("Failed to acquire bus access");
    }

    while (ros::ok()){
        std_msgs::String msg;
        msg.data = std::to_string(telaire.getPPM());
        ppm_publisher.publish(msg);
        usleep(500000);
    }

    return 0;
}
