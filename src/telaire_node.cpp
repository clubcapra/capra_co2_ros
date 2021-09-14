#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/String.h"

const int CO2_ADDR = 0x5A;        // default I2C slave address of CCS811
static const string CO2_DEV = "/dev/i2c-8"; // default I2C device file (check Jetson wiring)

/*I2C ADDRESS*/
const int CCS811_I2C_ADDRESS1 = 0x5A;
const int CCS811_I2C_ADDRESS2 = 0x5B;

const int CCS811_REG_STATUS = 0x00;
const int CCS811_REG_MEAS_MODE = 0x01;
const int CCS811_REG_ALG_RESULT_DATA = 0x02;
const int CCS811_REG_RAW_DATA = 0x03;
const int CCS811_REG_ENV_DATA = 0x05;
const int CCS811_REG_NTC = 0x06;
const int CCS811_REG_THRESHOLDS = 0x10;
const int CCS811_REG_BASELINE = 0x11;
const int CCS811_REG_HW_ID = 0x20;
const int CCS811_REG_HW_VERSION = 0x21;
const int CCS811_REG_FW_BOOT_VERSION = 0x23;
const int CCS811_REG_FW_APP_VERSION = 0x24;
const int CCS811_REG_INTERNAL_STATE = 0xA0;
const int CCS811_REG_ERROR_ID = 0xE0;
const int CCS811_REG_SW_RESET = 0xFF;

const int CCS811_BOOTLOADER_APP_ERASE = 0xF1;
const int CCS811_BOOTLOADER_APP_DATA = 0xF2;
const int CCS811_BOOTLOADER_APP_VERIFY = 0xF3;
const int CCS811_BOOTLOADER_APP_START = 0xF4;

const int CCS811_HW_ID = 0x81;

namespace DRIVE_MODE_t{
    enum Mode{
        Mode0, //Idle (Measurements are disabled in this mode)
        Mode1, //Constant power mode, IAQ measurement every second
        Mode2, //Pulse heating mode IAQ measurement every 10 seconds
        Mode3, //Low power pulse heating mode IAQ measurement every 60 seconds
        Mode4  //Constant power mode, sensor measurement every 250ms 1xx: Reserved modes (For future use)
    };
}
    
namespace Cycles
{
    enum Cycle
    {
        Closed,      //Idle (Measurements are disabled in this mode)
        Cycle_1s,    //Constant power mode, IAQ measurement every second
        Cycle_10s,   //Pulse heating mode IAQ measurement every 10 seconds
        Cycle_60s,   //Low power pulse heating mode IAQ measurement every 60 seconds
        Cycle_250s  //Constant power mode, sensor measurement every 250ms 1xx: Reserved modes (For future use)
    };
}

int main(void)
{

    int file;
    std::string filename = CO2_DEV;
    int ppmReading;

    if ((file = open(filename.c_str(), O_RDWR)) < 0)
    {
        ROS_INFO("Failed to open the bus.");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

    if (ioctl(file, I2C_SLAVE, CO2_ADDR) < 0)
    {
        ROS_INFO("Failed to acquire bus access and/or talk to slave.\n");

        exit(1);
    }

    // STARTING SETUP
    ROS_INFO("Starting setup\n");
    // Soft reset
    uint8_t buffer_soft_reset[5] = {CCS811_REG_SW_RESET, 0x11, 0xE5, 0x72, 0x8A};
    if( write(file, buffer_soft_reset, 5) != 5)
    {
        ROS_INFO("Failed to write SOFT RESET to I2C bus \n");
    }
    else
    {
        ROS_INFO("Sent soft reset\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ROS_INFO("SOft reset completed\n");
    }

    // Bootloader start
    uint8_t buffer_bootloader[1] = {CCS811_BOOTLOADER_APP_START};
    if( write(file, buffer_bootloader, 1) != 1)
    {
        ROS_INFO("Failed to write BOOTLOADER SETUP to I2C bus \n");
    }
    else
    {
        ROS_INFO("Sent bootloader setup\n");
    }

    // Set Measurement Mode
    uint8_t measurement[1] = {0};
    measurement[0] = (0 << 2) | (0 << 3) | (DRIVE_MODE_t::Mode4 << 4);
    ROS_INFO("Measurement mode set");
    uint8_t buffer_measurement_mode[2] = {CCS811_REG_MEAS_MODE, measurement[0]};
    if( write(file, buffer_measurement_mode, 2) != 2)
    {
        ROS_INFO("Failed to write MEASUREMENT MODE to I2C bus \n");
    }
    else
    {
        ROS_INFO("Sent Measurement mode\n");
    }

    // Set Temperature and Humidity
    double buffer_set_temp_hum[5] = {CCS811_REG_ENV_DATA, 50.5, 0, 24.5, 0};
    if( write(file, buffer_set_temp_hum, 5) != 5)
    {
        ROS_INFO("Failed to write TEMP AND HUM to I2C bus \n");
    }
    else
    {
        ROS_INFO("Set Temp and Hum\n");
    }

    // FIN SETUP
    ROS_INFO("End setup\n");
    uint8_t buffer_fetch_value[2] = {CCS811_REG_ALG_RESULT_DATA, 1};
    char buffer_read[8];
    ROS_INFO("Sending request for results\n");


    while (true)
    {
        if( write(file, buffer_fetch_value, 2) != 2)
        {
            ROS_INFO("Failed to write REQUEST FOR DATA to I2C bus \n");
        }
        else
        {
            usleep(100000);
            read(file, buffer_read, 8);
            // ROS_INFO("buffer_read value: %d, %d, %d, %d, %d, %d, %d, %d \n", buffer_read[0], buffer_read[1], buffer_read[2], buffer_read[3], buffer_read[4], buffer_read[5], buffer_read[6], buffer_read[7]);

            ppmReading = (((uint16_t)buffer_read[0] << 8) | (uint16_t)buffer_read[1]);

            ROS_INFO("C02 ppm: %d \n", ppmReading);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        ROS_INFO("*****\n");
    }

    return EXIT_SUCCESS;
}