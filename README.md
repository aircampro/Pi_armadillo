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

concept pics directory shows overview pictures of interfaces

I will very briefly explain the directories 
This shows the various areas covered for the libraires and API's in this repository
Please investigate the directory program for further details I will try to document more detail in time

GUI
- examples fro trend or control HMI using tkinter

ITL612 IR SENSOR
- ITL612 IR camera API

IR
- Infa Red controls to HVAC (toshiba panasonic mitsubishi) and audio (B&O) etc
  
PCIe Hailo
- PCIe interface for AI accelerator Hailo

QR code ANPR stuff
- read and write QR Codes and Number plates

SonyAlpha
- sony Alpha camera interface and to mavlink (qgroundcontrol) remote controls

Toshiba Telli
- toshiba telli pekat

ada_spark
- safe code can be written in ada spark

ble
- bluetooth low energy

LoRa
- LoRa radio link via usb port

ML
- Various machine learning techniques
  
boost_other
boost_prog
- examples using boost API

card reader
- various card readers Pasori and PCSC etc

cash
- various cash and card readsers for vending chraging etc

coAP
- coAP server / client

concept pics
- an overview concept of the functionality and interfaces provided

crow
- crow api webserver

data_manip
- various data manipulation and big data stuff

dicom
- x ray reader dicom files

drone
- drone controls and apis

echonet
- echonet for power charges

erlang
- erl;ang and elixir program examples

face_robot

fqueue
- queuing system with modbus slave/master

germination
- germination incubator using adafruit

hardware
- controlling direct GPIO inputs and outputs PWM
  
haskell
Glasgow Haskell programming

i2c_spi_gps
- i2c and spi examples

io_link
- io link examples

iot_enocean
- iot interfaces flask servers databases etc to enocean radio, other interfaces OPC BacNet etc 

knx
- knx and lighting

mimo
- adaptive control using mimo api

morse
- morse code genertor 

openCV
- openCV examples 

phoxi
- phoxi pointcloud lidar scanner interface

pi
- cpu optimization for very fast actions 

plc_robot_coms
- interfaces to PLCs and Robots and remote I/O's 

pluto
- interface code for Analog devices pluto SDR software defined radio

python mavlink
- mavlink for robot and drone remote controls

realsense
- realsense camera interface

resonon_pica_hyperspec_imager
- interface to camera controls 

rust_gripper
- rust code for gripper controls

rust_photon
- photon library wrapper for video/photo api

sonySRG300
- camera interface REST using boost

sound
- sound manipulation

tracker_LUT
- video tracker and lut correction

unitree_robos
- unitree robot examples

vending
- vending machine examples

video
- video and camera stuff v4l and ros interfaces etc

visual_odometry
- e.g. velodyne

visualize
- visual and lighting

working usb
- low level usb control
  
zigbee
- comms example via usb port, send a picture over zigbee from drone

