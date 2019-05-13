#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define CO2_ADDR 0x15        // default I2C slave address
#define CO2_DEV "/dev/i2c-1" // default I2C device file

int main(void)
{

    int file;
    std::string filename = "/dev/i2c-1";
    int ppmReading;

    if ((file = open(filename.c_str(), O_RDWR)) < 0)
    {
        printf("Failed to open the bus.");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

    if (ioctl(file, I2C_SLAVE, CO2_ADDR) < 0)
    {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

    char buffer_write[5] = {0x04, 0x13, 0x8b, 0x00, 0x01};
    char buffer_read[4];

    while (true)
    {
        if( write(file, buffer_write, 5) != 5)
        {
            printf("Failed to write to I2C bus \n");
            }
        else
        {
            usleep(100000);
            read(file, buffer_read, 4);

            printf("buffer_read value: %d, %d, %d, %d \n", buffer_read[0], buffer_read[1], buffer_read[2], buffer_read[3]);

            ppmReading = buffer_read[2] << 8 | buffer_read[3];

            printf("C02 ppm: %d \n", ppmReading);
        }

        usleep(500000);
        printf("*****\n");
    }

    return EXIT_SUCCESS;
}