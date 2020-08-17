# Humidity chamber controlled using Arduino Uno
This repository contains the code and the schematics for a simple humid chamber controlled using Arduino. The controlled humid chamber works as follow. The room, where the chamber is placed, is kept at low RH (e.g. 15%) and controlled temperature by means of the HVAC of the building. Humid air is generated trhough an ultrasonic humidifier and stored in a reservoir (e.g. a pipe). At the top of the pipe a centrifugal pump moves the humid air inside the chamber if needed.  Inside the chamber is placed  an humidity sensor (DHT11) connected to an Arduino Uno Rev. 3, that reads the humid sensor feedback and control the centrifugal pump using a motor controller for mantaining the RH desired value.

![Alt text](https://github.com/giacomomarchioro/humiditychambercontrol/blob/master/Climatic_chamber_drawing.svg "General schematics")

### Parts
-   [Arduino Uno Rev3](https://store.arduino.cc/arduino-uno-rev3)
-   [60W centrifugal pump](https://www.ebay.com/itm/Electric-Air-Pump-Power-Inflator-Blower-For-Boat-Car-Paddling-Pool-Bed-Mattress/114245834297?_trkparms=ispr%3D1&hash=item1a99951639:g:~M4AAOSwlZNe1cDW&amdata=enc%3AAQAFAAACgBaobrjLl8XobRIiIML1V4Imu%252Fn%252BzU5L90Z278x5ickkAgU0umhwUTmgTFbE5cu7zb%252BaWFB3yAO7zscmNogJK9oOacE2dNWEA%252B%252FEM%252B24Z6TUw7nT4%252FhglX3%252B1TW3LzV6DOLEhI27usq9%252F7kJs5vRNAe5KkRu9LsUch9AVciAWb8XSZjIZZbgmOg6NhkxHGMW9oB30ZxC9Jq0z8tQDudvE1zmRO6Ph0G%252FLPbzoHE%252F0k%252FwMJU5PuiBXVWJTpjLC3NG%252FQcApTPkH2B86zS6TsimBSM3RRTKHyHXb%252F%252BfWkUvaG%252BhCeMi50l0jN4SDh3lnzyZyNNPFd2Fe6Z3wRu9fFmDjOTFCW73YXghs%252B5j%252B0G%252Fb%252F%252BCsWwZp6dF23QHrzUq9plPuiNmpbP6oiWfp7j2oBektrxbg8Mjyzm0epJlQuYRK4KRAUYjQ4koweUTeq4Uwkd5BC10OswaZcfJoNuGtAMaboLw%252BwfQ6sARY8zDRZ7dCq45V437zmu80NKZOq%252FTfBs9zAzsOaEn73nJrz8%252FIRdw3Lo4%252BEZS702s%252FpNIn9yGnUkg4EiWs9pkSSpVvjAbcbvAgz1ZRSeKCnkcTUD3nKA0r5hdiVkH%252Bkx0Q3XsD3FNLmbbNFKw8XV4cUc8vx%252BW8IVK62Rmct00YB3MDnH%252BojvilMDsk2O152MvohyBdBIAzsLAI5bcN3WeLYxX5vXmdgaS69e2xQvOLibjQ6GgOeivw8bsXurHuL2XnmJrocnMcMCN%252FcDHur9wGeHMROdhxEj2P2IMnGHdCDh060xkDnc6TY5z6yk1U2AYOxnfTrW9y38PPhhroUenXN%252BTEr%252F61DyzzwNK2WMXBoXJAjd4j%252Bzy2Fs%253D%7Ccksum%3A1142458342979c1bb95104804bda9c3ab6522f0d2cfc%7Campid%3APL_CLK%7Cclp%3A2334524)
-   L298N (Dual motor controller) 
-   DHT11 (humidity sensor)
-   Humid air reservoir (e.g. 1 m pipe)
-   USB cable
-   Computer
-   Bosh BMP280 (Optional: Temperature and Pressure sensor)
### Instructions
Assembly the schematics as shown in the figure. Assure that the ultrasonic humidifer filled the reservoir before starting a new experiment. Using the Arduino IDE change `float target_HR =  50;` in `humiditychambercontrol/Arduino sketches/MicroclimateChamber_RH_T_P/MicroclimateChamber_RH_T_P.ino
` to the desire target relative humidty and compile the file (sketch) into the microcontroller. Turn on the pump. The pump will be on untile the target relative humidty is reached and will try to keep it as stable as possible.
