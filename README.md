# Humidity chamber controlled using Arduino Uno
This repository contains the code and the schematics for a simple humid chamber controlled using Arduino. The controlled humid chamber works as follow. The room, where the chamber is placed, is kept at low RH (e.g. 15%) and controlled temperature by means of the HVAC of the building. Humid air is generated trhough an ultrasonic humidifier and stored in a reservoir (e.g. a pipe). At the top of the pipe a centrifugal pump moves the humid air inside the chamber if needed.  Inside the chamber is placed  an humidity sensor (DHT11) connected to an Arduino Uno Rev. 3, that reads the humid sensor feedback and control the centrifugal pump using a motor controller for mantaining the RH desired value.
![Alt text](https://github.com/giacomomarchioro/humiditychambercontrol/blob/master/Climatic_chamber_drawing.svg "General schematics")

### Parts
-   Arduino Uno Rev3
-   60W centrifugal pump
-   L298N (Dual motor controller) 
-   DHT11 (humidity sensor)
-   Humid air reservoir (e.g. 1 m pipe)
-   USB cable
-   Computer
-   Bosh BMP280 (Optional: Temperature and Pressure sensor)
