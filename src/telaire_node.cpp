#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
//#include <linux/i2c-dev.h>
//#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <wiringSerial.h>

#define CO2_ADDR 0x15        // default I2C slave address
#define CO2_DEV "/dev/i2c-1" // default I2C device file
#define BAUD 9600

int main(int argc, char **argv)
{
    ros::init(argc, argv, "telaire");
    ros::NodeHandle n;
    ros::Publisher ppm_publisher = n.advertise<std_msgs::String>("ppm", 1);

    int file;
    std::string filename = "/dev/serial0"; // or "/dev/ttyAMA0"
    uint16_t ppmReading;

    if ((file = serialOpen(filename.c_str(), BAUD)) < 0)
    {
        ROS_INFO("Failed to open the bus.");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

    unsigned char buffer_write[8] = {CO2_ADDR, 0x04, 0x13, 0x8b, 0x00, 0x01, 0x46, 0x70};
    unsigned char buffer_read[7];

    int dataAvail = 0;

    ros::Rate r(10) //10 hz
    while (ros::ok())
    {
        std_msgs::String msg;
        
        serialPuts(file, buffer_write.c_str());

        usleep(50000);
        
        if (dataAvail = serialDataAvail(file) > 0){
            ROS_INFO("Failed to write to UART bus \n");
        } else {
            for(int i = 0; i < dataAvail; ++i){
                buffer_read[i] = serialGetchar(file);
            }
            
            if (buffer_read[0]==CO2_ADDR && buffer_read[1]==0x04 && buffer_read[2]==0x02)
            {
                ppmReading = (uint16_t)(buffer_read[3] << 8 | buffer_read[4]);
                msg.data = std::to_string(ppmReading);
                ppm_publisher.publish(msg);
                serialFlush(file);
            }
        }

        r.sleep();
    }

    serialClose(file);

    return EXIT_SUCCESS;
}
