# Pi_armadillo 
is a project of tools and applications that will run on a linux box such as a rasberry pi or armadillo
there are a series of data gathering routines which connect to spi i2c usb serial ethernet enocean BLE devices as well as hardwired pins
LoRa and enOcean remote sensors can be read such as temperature humidity, proximity switches on doors etc.
then there is a series of mechanisms to transfer than information via telemetry iOt cleint server mqtt broker/server OPC BacNet etc.. various 
databases such as influx, mongo or cloud storage like GCS Blob/spreadsheet, redis, nosql such as scyllia, rocks, streamers such as fluvio or dedicated iot like ubidots, ambient or machinst
it controls and reads periferals such as PLC/Robot, Camera, Motor Drives, and has inbuilt picture manipulation and picture tracking facilities
various PLC support, mitsubishi, omron, allen bradley, yokogawa, keyence, siemens, Robots yasakawa, kawasaki etc
Also has interface to GPS for position and interface to periferals such as OLED 2 line display.
Client - Server in rust with the client reading the width for an object it has read the label for and queries the MySQL db for the objects width before gripping it wqith a franka emika robot
controls a mitsubishi FX or Q series with modbus TCP over discord. Can query resnet from a picture or get information from Google Cloud API, haskell webserver examples for scotty and spock.
have fun it is still in development.....
