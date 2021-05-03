# capra_telaire_ros

capra_co2_ros is a node to interact with a Telaire T6713 CO2 sensor using I2C.


## Pin connections
The Telaire sensor has six pins. The output of each pin is described in the [telaire document](doc/AAS-916-142A-Telaire-T67xx-CO2-Sensor-022719-web.pdf) found in the doc file. Connection was made by matching the pins of the sensor to the pins of the Jetson following [its GPIO header pinout plan](https://www.jetsonhacks.com/nvidia-jetson-agx-xavier-gpio-header-pinout/). The pins #5 and #6 were left unplugged since we don't use PWM and pin #6 doesn't need to be connected.

Here we use the i2c communication bus, but the Telaire sensors allows you to use the pins with UART too.


## Testing on the Jetson AGX Xavier 
After pluggin the sensor to the Jetson, connect to the Jetson by ssh:
```ssh markhor­@IPADDR```
Replace IPADDR by the IP address of the Jetson. It is connected to the Capra network, so a simple netdiscover of the network should give you the IP address.

Launch the code with ```roslaunch capra_telaire_ros telaire.launch```

Make sure that the permissions of the pin you are connected to are right. You can check the address of the pin by executing the following command:
```sudo i2cdetect -y -r 8```

