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

const int CO2_ADDR = 0x5A;        // adresse par défaut du I2C slave de CCS811
const char* CO2_DEV = "/dev/i2c-8"; // fichier dev par défaut pour le I2C (vérifier branchdement du Jetson)

/*******    ADRESSES I2C    *******
 * Tiré de https://www.dropbox.com/sh/or1jzflapzbdepd/AAAGrCZgyjPOtNyLYNcyzL90a/Libraries/CCS811?dl=0&preview=CCS811.h&subfolder_nav_tracking=1
*/
const int CCS811_REG_MEAS_MODE = 0x01;
const int CCS811_REG_ALG_RESULT_DATA = 0x02;
const int CCS811_REG_ENV_DATA = 0x05;

// Constantes pour le soft reset
const int CCS811_REG_BASELINE = 0x11;
const int CCS811_REG_SW_RESET = 0xFF;
const int CCS811_REG_RESET_INDEX2 = 0xE5;
const int CCS811_REG_RESET_INDEX3 = 0x72;
const int CCS811_REG_RESET_INDEX4 = 0x8A;
const int CCS811_BOOTLOADER_APP_START = 0xF4;

/* 
*   Voir https://www.dropbox.com/sh/or1jzflapzbdepd/AAAGrCZgyjPOtNyLYNcyzL90a/Libraries/CCS811?dl=0&preview=CCS811.h&subfolder_nav_tracking=1
*   pour plus de Modes si nécessaire
*/
namespace DRIVE_MODE_t{
    enum Mode{
        Mode4  // Mode alimentation constante, mesures effectuées chaque 250 ms
    };
}

/*
*   Tiré de https://www.dropbox.com/sh/or1jzflapzbdepd/AAAGrCZgyjPOtNyLYNcyzL90a/Libraries/CCS811?dl=0&preview=CCS811.cpp&subfolder_nav_tracking=1
*/
int main(int argc, char *argv[])
{

    int file;
    std::string filename = CO2_DEV;
    int ppmReading;
    bool log_measurement_enabled = false;

    if ((file = open(filename.c_str(), O_RDWR)) < 0)
    {
        ROS_INFO("Impossible d'ouvrir le bus...");
        /* ERROR HANDLING; vérifier errno pour voir le problème */
        exit(1);
    }

    if (ioctl(file, I2C_SLAVE, CO2_ADDR) < 0)
    {
        ROS_INFO("Impossible d'accéder au bus et/ou de parler au slave.\n");

        exit(1);
    }

    // DÉBUT INITIALISATION
    ROS_INFO(" *** Début initialisation *** \n");
    // Soft reset
    uint8_t buffer_soft_reset[5] = {CCS811_REG_SW_RESET, CCS811_REG_BASELINE, CCS811_REG_RESET_INDEX2, CCS811_REG_RESET_INDEX3, CCS811_REG_RESET_INDEX4};
    if( write(file, buffer_soft_reset, 5) != 5)
    {
        ROS_INFO("Impossible d'envoyer le SOFT RESET au bus I2C.\n");
    }
    else
    {
        ROS_INFO("SOFT RESET envoyé.\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ROS_INFO("SOFT RESET complété.\n");
    }

    // Bootloader
    uint8_t buffer_bootloader[1] = {CCS811_BOOTLOADER_APP_START};
    if( write(file, buffer_bootloader, 1) != 1)
    {
        ROS_INFO("Impossible d'écrire l'initialisation du BOOTLOADER au bus I2C.\n");
    }
    else
    {
        ROS_INFO("Initialisation du BOOTLOADER envoyé.\n");
    }

    // Mode de mesure - Measurement Mode
    uint8_t measurement[1] = {0};
    measurement[0] = (0 << 2) | (0 << 3) | (DRIVE_MODE_t::Mode4 << 4);
    ROS_INFO("MEASUREMENT MODE mis en place.");
    uint8_t buffer_measurement_mode[2] = {CCS811_REG_MEAS_MODE, measurement[0]};
    if( write(file, buffer_measurement_mode, 2) != 2)
    {
        ROS_INFO("Impossible d'écrire le MEASUREMENT MODE au bus I2C.\n");
    }
    else
    {
        ROS_INFO("MEASUREMENT MODE envoyé.\n");
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

    // FIN INITIALISATION
    ROS_INFO(" *** Fin initialistion *** \n");
    uint8_t buffer_fetch_value[2] = {CCS811_REG_ALG_RESULT_DATA, 1};
    char buffer_read[8];
    ROS_INFO("Envoi de la requête pour récolter les données...\n");


    while (true)
    {
        if( write(file, buffer_fetch_value, 2) != 2)
        {
            ROS_INFO("Impossible d'envoyer la requête au bus I2C pour récolter les données.\n");
        }
        else
        {
            usleep(100000);
            read(file, buffer_read, 8);
            if (argc == 4 && strcmp(argv[1],"true")==0)
            {
                ROS_INFO("buffer_read value: %d, %d, %d, %d, %d, %d, %d, %d \n", buffer_read[0], buffer_read[1], buffer_read[2], buffer_read[3], buffer_read[4], buffer_read[5], buffer_read[6], buffer_read[7]);
            }

            ppmReading = (((uint16_t)buffer_read[0] << 8) | (uint16_t)buffer_read[1]);

            ROS_INFO("C02 ppm: %d \n", ppmReading);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        ROS_INFO("*****\n");
    }

    return EXIT_SUCCESS;
}