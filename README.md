# capra_telaire_ros

capra_co2_ros is a node to interact with a KS0457 keyestudio CCS811 Carbon Dioxide Air Quality sensor using I2C.


## Documentation
Documentation can be found on the [wiki page](https://wiki.keyestudio.com/KS0457_keyestudio_CCS811_Carbon_Dioxide_Air_Quality_Sensor) of the sensor. The [code of the library](https://www.dropbox.com/sh/or1jzflapzbdepd/AAAGrCZgyjPOtNyLYNcyzL90a/Libraries/CCS811?dl=0&preview=CCS811.cpp) was reproduced for a Jetson as we do not use an Arduino.


## Pin connections
The CCS811 sensor has six pins. According to the wiki page, we only need to connect five pins: the ground (GND), the VCC, the data (SDA), the clock (SCL) and the WAKE pin. The WAKE needs to be connected to ground and the sensor feeds off 5V.


## Testing on the Jetson AGX Xavier 
After plugging the sensor to the Jetson, connect to the Jetson by ssh:
```ssh markhor­@IPADDR```
Replace IPADDR by the IP address of the Jetson. It is connected to the Capra network, so a simple netdiscover of the network should give you the IP address.

Launch the code with ```roslaunch capra_telaire_ros telaire.launch log_on:="false"```

You can show the details of the buffer_read value by changing the value of the argument log_on to true: ```log_on:="true"```

Make sure that the permissions of the pin you are connected to are right. You can check the address of the pin by executing the following command:
```sudo i2cdetect -y -r 8```

Seeing 5A after execution of the command indicates the wiring is ok.
