# Pi_armadillo 
is a project of tools and applications that will run on a linux box such as a rasberry pi or armadillo
there are a series of data gathering routines which connect to spi i2c usb serial ethernet enocean BLE devices as well as hardwired pins
LoRa and enOcean remote sensors can be read such as temperature humidity, proximity switches on doors etc.
then there is a series of mechanisms to transfer than information via telemetry iOt cleint server mqtt broker/server OPC BacNet etc.. various AWS AZURE YANDEX uBiDots machinist etc..
databases such as influx, mongo or cloud storage like GCS Blob/spreadsheet, redis, nosql such as scyllia, rocks, streamers such as fluvio or dedicated iot like ubidots, ambient or machinst
it controls and reads periferals such as PLC/Robot, Camera, Motor Drives, and has inbuilt picture manipulation and picture tracking facilities
various PLC support, mitsubishi, omron, allen bradley, yokogawa, keyence, siemens, Robots yasakawa, kawasaki universal robots UR3, UR5 and UR10, abb robotics etc
Also has interface to GPS for position and interface to periferals such as OLED 2 line display.
Client - Server in rust with the client reading the width for an object it has read the label for and queries the MySQL db for the objects width before gripping it wqith a franka emika robot
controls a Mitsubishi FX or Q series with modbus TCP over discord. UR Robot to Omron FINS TCP in rust. a little bit of erlang for queueing.
radio links BLE EnOcean LoRa Zigbee etc, usb interfaces, serial. Franka Emika gripper query to webserver of label read using tesseract to get width then grip object.
Can query resnet from a picture or get information from Google Cloud API, haskell webserver examples for scotty and spock. Interface to aldebaran pepper robot from GPT-3
eSSP and CCtalk protocol examples for communication with money machines and gaming dispensers AES encrpytion and Diffie Hellman key exchange
visual odometry example allowing the feature detection algorithm to be changed.
.................have fun it is still in development.....
